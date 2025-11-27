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


class GoalMovementMover6 : public rclcpp::Node
{
public:
	GoalMovementMover6()
		: Node("GoalMovementMover6"), count_(0)
	{
		// RCLCPP_INFO(this->get_logger(),"Constructor");
		current_joints.resize(6, 0.0f);
		demanded_joints.resize(6, 0.0f);
		cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		tm_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions options;
		options.callback_group = cb_group_;
		publisherJointPosition_ = this->create_publisher<control_msgs::msg::JointJog>("/JointJog", 10);
		subscriptionJointPosition_ = this->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 10, std::bind(&GoalMovementMover6::topic_jointStatesCallback, this, _1), options);
		subscriptionJointDemands_ = this->create_subscription<std_msgs::msg::Float32MultiArray>("/joint_demands", 10, std::bind(&GoalMovementMover6::topic_jointDemandsCallback, this, _1), options);
		timer_ = this->create_wall_timer(100ms, std::bind(&GoalMovementMover6::timer_callback, this), tm_group_);
	}

private:
	void timer_callback()
	{
		move_my_robot();
	}
	void topic_jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		if (msg->position.size() < 6) {
        	return;
    	}

		for (int i = 0; i < 6; i++)
		{
			current_joints[i] = msg->position[i];
		}
		known_states = true;
		// RCLCPP_INFO(this->get_logger(),"Received State %f\t%f\t%f\t%f\t%f\t%f", joint1_angle, joint2_angle, joint3_angle, joint4_angle, joint5_angle, joint6_angle);
	}
	void topic_jointDemandsCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
	{
		if (msg->data.size() < 6) {
        	return;
    	}

		for (int i = 0; i < 6; i++)
		{
			demanded_joints[i] = msg->data[i];
		}
		known_demands = true;
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

		double tolerance = 0.05;
		double max_speed = 0.5;

		std::vector<double> errors(6);
		std::vector<double> velocities(6, 0.0);
		double max_err = 0.0;

		// Calculate All Joint Errors -> Find Max Error
		for (int i = 0; i < 6; i++)
		{
			errors[i] = demanded_joints[i] - current_joints[i];
			max_err = std::max(abs(errors[i]), max_err);
		}

		// Stop if Max Error < Tolerance
		if (max_err < tolerance)
		{
			send_msg(velocities);
			return;
		}

		// Stop if Calculated Time to Target < 0.001
		double time = max_err / max_speed;
		if (time < 0.001)
		{
			send_msg(velocities);
			return;
		}

		// Calculate Proportional Velocities for All Joints
		for (int i = 0; i < 6; i++)
		{
			velocities[i] = errors[i] / time;
		}

		send_msg(velocities);
	}

	bool known_states = false;
	bool known_demands = false;
	std::vector<float> current_joints;
	std::vector<float> demanded_joints;
	rclcpp::CallbackGroup::SharedPtr cb_group_;
	rclcpp::CallbackGroup::SharedPtr tm_group_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisherJointPosition_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriptionJointPosition_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionJointDemands_;
	unsigned long long state = 0;
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
