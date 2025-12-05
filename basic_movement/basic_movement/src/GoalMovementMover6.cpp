// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <functional>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <sstream>
using std::placeholders::_1;
using namespace std::chrono_literals;

struct PolynomialCoeffs
{
	double a0, a1, a2, a3, a4, a5;
};

class GoalMovementMover6 : public rclcpp::Node
{
public:
	GoalMovementMover6()
		: Node("GoalMovementMover6"), count_(0)
	{
		// RCLCPP_INFO(this->get_logger(),"Constructor");
		kp = 3.5;  // Proportional gain for position error correction
		ki = 0.1;  // Integral gain
		kd = 0.5;  // Derivative gain
		dt = 0.01; // Time step for control loop (seconds)
		errors_prev.resize(6, 0.0f);
		errors_integral.resize(6, 0.0f);

		integral_limit = 1.0 / ki;

		current_joints.resize(6, 0.0f);
		current_velocities.resize(6, 0.0f);
		demanded_joints.resize(6, 0.0f);
		coeffs.resize(6);
		cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		tm_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions options;
		options.callback_group = cb_group_;
		publisherJointPosition_ = this->create_publisher<control_msgs::msg::JointJog>("/JointJog", 10);
		subscriptionJointPosition_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&GoalMovementMover6::topic_jointStatesCallback, this, _1), options);
		subscriptionJointDemands_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/joint_demands", 10, std::bind(&GoalMovementMover6::topic_jointDemandsCallback, this, _1), options);
		timer_ = this->create_wall_timer(10ms, std::bind(&GoalMovementMover6::timer_callback, this), tm_group_);
	}

private:
	PolynomialCoeffs get_coeffs(double q_start, double q_target, double v_start, double T)
	{
		PolynomialCoeffs coeffs;
		double diff = q_target - q_start;

		coeffs.a0 = q_start;
		coeffs.a1 = v_start; // NEW: Start velocity is no longer hardcoded to 0
		coeffs.a2 = 0.0;	 // Assuming 0 start acceleration

		// Calculating based on assumption: v_final = 0 & accel_final = 0
		coeffs.a3 = (10.0 * diff) / std::pow(T, 3) - (6.0 * v_start) / std::pow(T, 2);
		coeffs.a4 = (-15.0 * diff) / std::pow(T, 4) + (8.0 * v_start) / std::pow(T, 3);
		coeffs.a5 = (6.0 * diff) / std::pow(T, 5) - (3.0 * v_start) / std::pow(T, 4);

		return coeffs;
	}

	double get_position(const PolynomialCoeffs &coeffs, double t)
	{
		return coeffs.a0 + coeffs.a1 * t + coeffs.a2 * std::pow(t, 2) + coeffs.a3 * std::pow(t, 3) + coeffs.a4 * std::pow(t, 4) + coeffs.a5 * std::pow(t, 5);
	}

	double get_velocity(const PolynomialCoeffs &coeffs, double t)
	{
		return coeffs.a1 + 2 * coeffs.a2 * t + 3 * coeffs.a3 * std::pow(t, 2) + 4 * coeffs.a4 * std::pow(t, 3) + 5 * coeffs.a5 * std::pow(t, 4);
	}

	void timer_callback()
	{
		move_my_robot();
	}

	void topic_jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		if (msg->position.size() < 6)
		{
			return;
		}

		double velocity_flag = false;
		// Get current velocity if available
		if (msg->velocity.size() >= 6)
		{
			velocity_flag = true;
		}

		for (int i = 0; i < 6; i++)
		{
			current_joints[i] = msg->position[i];
			current_velocities[i] = (velocity_flag) ? msg->velocity[i] : 0.0;
		}
		known_states = true;
		// RCLCPP_INFO(this->get_logger(),"Received State %f\t%f\t%f\t%f\t%f\t%f", joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle);
	}

	void topic_jointDemandsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
	{
		if (msg->data.size() < 6)
		{
			return;
		}

		bool demands_changed = false;
		for (int i = 0; i < 6; i++)
		{
			if (std::abs(demanded_joints[i] - msg->data[i]) > 1e-6)
			{
				demanded_joints[i] = msg->data[i];
				demands_changed = true;
			}
		}

		if (demands_changed && known_states)
		{
			this->start_new_trajectory();
		}

		known_demands = true;
	}

	void start_new_trajectory()
	{
		std::lock_guard<std::mutex> lock(trajectory_mutex); // Lock Data

		// Calculate maximum error to determine trajectory duration double max_error = 0.0;
		double max_error = 0.0;
		for (int i = 0; i < 6; i++)
		{
			double error = std::abs(demanded_joints[i] - current_joints[i]);
			max_error = std::max(error, max_error);
		}

		// Set trajectory duration based on max error
		// Adjust this factor to control overall speed
		double max_speed = 0.25;									// rad/s
		trajectory_duration = std::max(max_error / max_speed, 5.0); // Minimum 5 seconds

		// Calculate coefficients for each joint
		for (int i = 0; i < 6; i++)
		{
			coeffs[i] = get_coeffs(
				current_joints[i],
				demanded_joints[i],
				current_velocities[i],
				trajectory_duration);

			// Reset PID Memory
			errors_integral[i] = 0.0;
			errors_prev[i] = 0.0;
		}

		// Record start time
		trajectory_start_time = this->now();
		trajectory_active = true;

		RCLCPP_INFO(this->get_logger(),
					"Starting new trajectory, duration: %.2f seconds",
					trajectory_duration);
	}

	void send_msg(const std::vector<double> &vels)
	{
		auto msg = control_msgs::msg::JointJog();
		msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
		msg.velocities = vels;

		publisherJointPosition_->publish(msg);
	}

	void move_my_robot()
	{
		if (!known_states || !known_demands)
		{
			return;
		}

		std::lock_guard<std::mutex> lock(trajectory_mutex);

		std::vector<double> velocities(6, 0.0);

		if (!trajectory_active)
		{
			// No active trajectory, send zero velocities
			send_msg(velocities);
			return;
		}

		// Calculate elapsed time
		double elapsed = (this->now() - trajectory_start_time).seconds();

		// Check if trajectory is complete
		if (elapsed >= trajectory_duration)
		{
			// Trajectory finished
			trajectory_active = false;

			// Check if we're close enough to target
			double max_error = 0.0;
			for (int i = 0; i < 6; i++)
			{
				double error = std::abs(demanded_joints[i] - current_joints[i]);
				max_error = std::max(error, max_error);
			}

			if (max_error < 0.05)
			{
				RCLCPP_INFO(this->get_logger(), "Trajectory complete, at target");
			}
			else
			{
				RCLCPP_WARN(this->get_logger(),
							"Trajectory complete but error remains: %.4f rad",
							max_error);
			}

			send_msg(velocities);
			return;
		}

		// Calculate desired velocities from quintic trajectory
		for (int i = 0; i < 6; i++)
		{
			double error = get_position(coeffs[i], elapsed) - current_joints[i];

			// Proportional Term
			double P = kp * error;

			// Integral Term
			errors_integral[i] += error * dt;
			errors_integral[i] = std::clamp(errors_integral[i], -integral_limit, integral_limit);
			double I = ki * errors_integral[i];

			// Derivative Term
			double D = kd * (error - errors_prev[i]) / dt;

			// Feedforward Term
			double feedforward_vel = get_velocity(coeffs[i], elapsed);

			velocities[i] = feedforward_vel + P + I + D;
			errors_prev[i] = error;
		}

		send_msg(velocities);
	}

	std::mutex trajectory_mutex;

	double kp;
	double ki;
	double kd;
	double dt;
	float integral_limit;

	bool known_states = false;
	bool known_demands = false;
	bool trajectory_active = false;

	std::vector<float> current_joints;
	std::vector<float> current_velocities;
	std::vector<float> demanded_joints;
	std::vector<PolynomialCoeffs> coeffs;
	std::vector<float> errors_prev;
	std::vector<float> errors_integral;

	rclcpp::Time trajectory_start_time;
	double trajectory_duration;

	rclcpp::CallbackGroup::SharedPtr cb_group_;
	rclcpp::CallbackGroup::SharedPtr tm_group_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisherJointPosition_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriptionJointPosition_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionJointDemands_;
	size_t count_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	// auto node = rclcpp::Node::make_shared("GoalMovementMover6");
	auto node = std::make_shared<GoalMovementMover6>();
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(node);
	executor.spin();
	// rclcpp::spin(std::make_shared<GoalMovementMover6>());
	rclcpp::shutdown();
	return 0;
}
