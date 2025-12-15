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
#include <algorithm>
#include <cmath>
#include <control_msgs/msg/joint_jog.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sstream>
using std::placeholders::_1;
using namespace std::chrono_literals;

// Struct Definitions
struct PolynomialCoeffs
{
	double a0, a1, a2, a3, a4, a5;
};

struct ViaPoint
{
	std::vector<double> positions; // Joint positions [6]
	double segment_duration;	   // Time from previous via point
	double blend_time;			   // Total blend duration at this via point
};

struct TrajectorySegment
{
	enum class Type
	{
		LINEAR,
		BLEND
	};
	Type type;
	double start_time;					 // Global trajectory time when segment begins
	double duration;					 // Duration of this segment
	std::vector<double> start_positions; // LINEAR: start pos
	std::vector<double> velocities;		 // LINEAR: constant vel
	std::vector<double> blend_start_pos; // BLEND: pos at blend start
	std::vector<double> incoming_vel;	 // BLEND: velocity entering
	std::vector<double> accelerations;	 // BLEND: constant accel
};

struct MultiSegmentTrajectory
{
	std::vector<ViaPoint> via_points;
	std::vector<TrajectorySegment> segments;
	double total_duration;
};

enum class TrajectoryState
{
	IDLE,
	SINGLE_SEGMENT,
	MULTI_SEGMENT
};

class GoalMovementMover6 : public rclcpp::Node
{
public:
	GoalMovementMover6()
		: Node("GoalMovementMover6"), count_(0)
	{
		// PID gains (tuned for faster convergence)
		kp_ = 10.0; // Proportional gain
		ki_ = 1.5;	// Integral gain
		kd_ = 0.1;	// Derivative gain
		dt_ = 0.01; // Time step for control loop (seconds)
		errors_prev_.resize(6, 0.0f);
		errors_integral_.resize(6, 0.0f);
		integral_limit_ = 2.5; // Fixed anti-windup limit

		// Declare ROS parameters for trajectory configuration
		this->declare_parameter("max_velocity", 0.25);
		this->declare_parameter("max_acceleration", 0.5);
		this->declare_parameter("max_jerk", 1.0);
		this->declare_parameter("min_blend_time", 0.5);
		this->declare_parameter("min_segment_time", 0.5);

		// Load parameters
		max_velocity_ = this->get_parameter("max_velocity").as_double();
		max_acceleration_ = this->get_parameter("max_acceleration").as_double();
		max_jerk_ = this->get_parameter("max_jerk").as_double();
		min_blend_time_ = this->get_parameter("min_blend_time").as_double();
		min_segment_time_ = this->get_parameter("min_segment_time").as_double();

		// Initialize state vectors
		current_joints_.resize(6, 0.0f);
		current_velocities_.resize(6, 0.0f);
		demanded_joints_.resize(6, 0.0f);
		coeffs_.resize(6);
		joint_active_.resize(6, false);

		// Create callback groups
		cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		tm_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		rclcpp::SubscriptionOptions options;
		options.callback_group = cb_group_;

		// Create publisher
		publisherJointPosition_ = this->create_publisher<control_msgs::msg::JointJog>("/JointJog", 10);
		publisherDesiredJointStates_ = this->create_publisher<sensor_msgs::msg::JointState>("/desired_joint_states", 10);

		// Create subscriptions
		subscriptionJointPosition_ = this->create_subscription<sensor_msgs::msg::JointState>(
			"/joint_states", 10,
			std::bind(&GoalMovementMover6::topic_jointStatesCallback, this, _1), options);
		subscriptionJointDemands_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
			"/joint_demands", 10,
			std::bind(&GoalMovementMover6::topic_jointDemandsCallback, this, _1), options);
		subscriptionJointTrajectory_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
			"/joint_trajectory", 10,
			std::bind(&GoalMovementMover6::topic_jointTrajectoryCallback, this, _1), options);

		// Create timer for control loop (wall timer fires regardless of use_sim_time)
		timer_ = this->create_wall_timer(10ms,
										 std::bind(&GoalMovementMover6::timer_callback, this), tm_group_);
	}

private:
	// ============== Single-segment trajectory polynomial generation ==============

	// Get desired coefficients for a joint to achieve target trajectory
	PolynomialCoeffs get_coeffs(double q_start, double q_target, double v_start, double T)
	{
		PolynomialCoeffs coeffs;
		double diff = q_target - q_start;

		coeffs.a0 = q_start;
		coeffs.a1 = v_start; // Start velocity is no longer hardcoded to 0
		coeffs.a2 = 0.0;	 // Assuming 0 start acceleration

		// Calculating based on assumption: v_final = 0 & accel_final = 0
		coeffs.a3 = (10.0 * diff) / std::pow(T, 3) - (6.0 * v_start) / std::pow(T, 2);
		coeffs.a4 = (-15.0 * diff) / std::pow(T, 4) + (8.0 * v_start) / std::pow(T, 3);
		coeffs.a5 = (6.0 * diff) / std::pow(T, 5) - (3.0 * v_start) / std::pow(T, 4);

		return coeffs;
	}

	// Get desired joint position and velocity for each joint; Given the current elapsed time
	void evaluate_single_trajectory(double t, std::vector<double> &desired_positions, std::vector<double> &desired_velocities)
	{
		desired_positions.resize(6);
		desired_velocities.resize(6);

		for (int j = 0; j < 6; j++)
		{
			desired_positions[j] = coeffs_[j].a0 + coeffs_[j].a1 * t + coeffs_[j].a2 * std::pow(t, 2) + coeffs_[j].a3 * std::pow(t, 3) + coeffs_[j].a4 * std::pow(t, 4) + coeffs_[j].a5 * std::pow(t, 5);
			desired_velocities[j] = coeffs_[j].a1 + 2 * coeffs_[j].a2 * t + 3 * coeffs_[j].a3 * std::pow(t, 2) + 4 * coeffs_[j].a4 * std::pow(t, 3) + 5 * coeffs_[j].a5 * std::pow(t, 4);
		}
	}

	// ============== Multi-segment trajectory generation ==============

	// Calculate duration for each segment in trajectory
	// One segment consists of a half blend + linear + half blend sections
	void compute_segment_durations(std::vector<ViaPoint> &via_points)
	{
		via_points[0].segment_duration = 0.0; // First point has no duration

		for (size_t i = 1; i < via_points.size(); i++)
		{
			// Find maximum joint error
			double max_err = 0.0;
			for (int j = 0; j < 6; j++)
			{
				double err = std::abs(via_points[i].positions[j] - via_points[i - 1].positions[j]);
				max_err = std::max(max_err, err);
			}

			// Duration based on max velocity
			double duration = max_err / max_velocity_;

			// Apply duration, keeping in mind minimum segment time
			via_points[i].segment_duration = std::max(duration, min_segment_time_);
		}
	}

	// Calculate total blend time for each via point in the trajectory (Based on maximum acceleration & jerk)
	void compute_blend_durations(std::vector<ViaPoint> &via_points)
	{
		// First and last points have zero blend time (start/end at rest)
		via_points.front().blend_time = 0.0;
		via_points.back().blend_time = 0.0;

		// Calculate blend times for remaining via points (Based on configured limits)
		for (size_t k = 1; k < via_points.size() - 1; k++)
		{
			double max_blend_time = 0.0;

			for (int j = 0; j < 6; j++)
			{
				// Compute incoming and outgoing velocities for joint j
				double v_in = (via_points[k].positions[j] - via_points[k - 1].positions[j]) / via_points[k].segment_duration;
				double v_out = (via_points[k + 1].positions[j] - via_points[k].positions[j]) / via_points[k + 1].segment_duration;

				double delta_v = std::abs(v_out - v_in);

				if (delta_v < 1e-9)
					continue; // No velocity change needed

				// Acceleration-limited blend time
				double t_accel = delta_v / max_acceleration_;

				// Jerk-limited blend time: t_blend = sqrt(2 * delta_v / max_jerk)
				double t_jerk = std::sqrt(2.0 * delta_v / max_jerk_);

				// Use the more restrictive constraint
				double t_blend_j = std::max(t_accel, t_jerk);
				max_blend_time = std::max(max_blend_time, t_blend_j);
			}

			// Apply blend time, keeping in mind minimum blend time
			via_points[k].blend_time = std::max(max_blend_time, min_blend_time_);
		}
	}

	// Adjust calculated blend times to ensure no blend overlapping (linear time >= 0)
	bool adjust_blend_durations(std::vector<ViaPoint> &via_points)
	{
		// Ensure blend regions don't overlap
		for (size_t k = 1; k < via_points.size(); k++)
		{
			double half_blend_prev = via_points[k - 1].blend_time / 2.0;
			double half_blend_curr = via_points[k].blend_time / 2.0;
			double linear_time = via_points[k].segment_duration - half_blend_prev - half_blend_curr;

			if (linear_time < 0.0)
			{
				// Blends overlap: need to increase segment duration
				double required_duration = half_blend_prev + half_blend_curr + 0.1; // 0.1s margin
				via_points[k].segment_duration = required_duration;

				RCLCPP_WARN(this->get_logger(),
							"Segment %zu duration increased to %.2f to avoid blend overlap",
							k, required_duration);
			}
		}
		return true;
	}

	// Build full multi-segment trajectory
	MultiSegmentTrajectory build_trajectory_segments(const std::vector<ViaPoint> &via_points)
	{
		MultiSegmentTrajectory trajectory;
		trajectory.via_points = via_points;
		trajectory.segments.clear();

		double global_time = 0.0;

		for (size_t k = 0; k < via_points.size(); k++)
		{
			// === LINEAR SECTION from k to k+1 ===
			if (k < via_points.size() - 1)
			{
				double half_blend_k = via_points[k].blend_time / 2.0;
				double half_blend_next = via_points[k + 1].blend_time / 2.0;
				double linear_duration = via_points[k + 1].segment_duration - half_blend_k - half_blend_next;

				if (linear_duration > 0.0)
				{
					TrajectorySegment linear;
					linear.type = TrajectorySegment::Type::LINEAR;
					linear.start_time = global_time + half_blend_k;
					linear.duration = linear_duration;

					linear.start_positions.resize(6);
					linear.velocities.resize(6);

					for (int j = 0; j < 6; j++)
					{
						double v = (via_points[k + 1].positions[j] - via_points[k].positions[j]) / via_points[k + 1].segment_duration;

						// Position at linear segment start (after blend k)
						linear.start_positions[j] = via_points[k].positions[j] + v * half_blend_k;
						linear.velocities[j] = v;
					}

					trajectory.segments.push_back(linear);
				}
			}

			// === BLEND SECTION at via point k+1 (except last point) ===
			if (k < via_points.size() - 2)
			{
				size_t blend_idx = k + 1;
				if (via_points[blend_idx].blend_time > 0)
				{
					TrajectorySegment blend;
					blend.type = TrajectorySegment::Type::BLEND;

					// Blend starts half-blend before reaching the via point
					// time_at_via is when we reach via point k+1 (not global_time which is at k)
					double time_at_via = global_time + via_points[blend_idx].segment_duration;
					blend.start_time = time_at_via - via_points[blend_idx].blend_time / 2.0;
					blend.duration = via_points[blend_idx].blend_time;

					blend.blend_start_pos.resize(6);
					blend.incoming_vel.resize(6);
					blend.accelerations.resize(6);

					for (int j = 0; j < 6; j++)
					{
						double v_in = (via_points[blend_idx].positions[j] - via_points[blend_idx - 1].positions[j]) / via_points[blend_idx].segment_duration;
						double v_out = (via_points[blend_idx + 1].positions[j] - via_points[blend_idx].positions[j]) / via_points[blend_idx + 1].segment_duration;

						blend.incoming_vel[j] = v_in;
						blend.accelerations[j] = (v_out - v_in) / blend.duration;

						// Position at blend start (half-blend before via point)
						blend.blend_start_pos[j] = via_points[blend_idx].positions[j] - v_in * (blend.duration / 2.0);
					}

					trajectory.segments.push_back(blend);
				}
			}
			global_time += via_points[k + 1].segment_duration;
		}

		trajectory.total_duration = global_time;

		// Sort segments by start time
		std::sort(trajectory.segments.begin(), trajectory.segments.end(),
				  [](const TrajectorySegment &a, const TrajectorySegment &b)
				  {
					  return a.start_time < b.start_time;
				  });

		return trajectory;
	}

	// ============== Trajectory evaluation ==============

	const TrajectorySegment *find_active_segment(double elapsed_time) const
	{
		for (const auto &seg : multi_trajectory_.segments)
		{
			if (elapsed_time >= seg.start_time &&
				elapsed_time < seg.start_time + seg.duration)
			{
				return &seg;
			}
		}
		return nullptr; // Past all segments
	}

	void evaluate_multi_trajectory(double elapsed,
								   std::vector<double> &desired_positions,
								   std::vector<double> &desired_velocities)
	{
		desired_positions.resize(6);
		desired_velocities.resize(6);

		const TrajectorySegment *seg = find_active_segment(elapsed);

		if (seg == nullptr)
		{
			// Past end of trajectory: hold final position
			const ViaPoint &final_pt = multi_trajectory_.via_points.back();
			for (int j = 0; j < 6; j++)
			{
				desired_positions[j] = final_pt.positions[j];
				desired_velocities[j] = 0.0;
			}
			return;
		}

		double t_local = elapsed - seg->start_time; // Time within segment

		if (seg->type == TrajectorySegment::Type::LINEAR)
		{
			for (int j = 0; j < 6; j++)
			{
				desired_positions[j] = seg->start_positions[j] + seg->velocities[j] * t_local;
				desired_velocities[j] = seg->velocities[j];
			}
		}
		else if (seg->type == TrajectorySegment::Type::BLEND)
		{
			for (int j = 0; j < 6; j++)
			{
				desired_positions[j] = seg->blend_start_pos[j] + seg->incoming_vel[j] * t_local + 0.5 * seg->accelerations[j] * std::pow(t_local, 2);
				desired_velocities[j] = seg->incoming_vel[j] + seg->accelerations[j] * t_local;
			}
		}
	}

	// ============== Callbacks ==============

	void timer_callback()
	{
		move_my_robot();
	}

	void topic_jointStatesCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
	{
		// Joint names we're looking for (in order: joint1=index0, joint2=index1, etc.)
		static const std::vector<std::string> joint_names = {
			"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

		bool velocity_flag = (msg->velocity.size() >= msg->position.size());

		// Match joints by name, not by index
		for (size_t i = 0; i < joint_names.size(); i++)
		{
			for (size_t j = 0; j < msg->name.size(); j++)
			{
				if (msg->name[j] == joint_names[i])
				{
					if (j < msg->position.size())
					{
						current_joints_[i] = msg->position[j];
					}
					if (velocity_flag && j < msg->velocity.size())
					{
						current_velocities_[i] = msg->velocity[j];
					}
					break;
				}
			}
		}

		known_states_ = true;
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
			if (std::abs(demanded_joints_[i] - msg->data[i]) > 1e-6)
			{
				demanded_joints_[i] = msg->data[i];
				demands_changed = true;
			}
		}

		if (!demands_changed)
		{
			RCLCPP_WARN(this->get_logger(),
						"No changes in joint demands; Not starting trajectory");
			return;
		}
		if (!known_states_)
		{

			RCLCPP_WARN(this->get_logger(),
						"Cannot start trajectory: joint states not yet received");
			return;
		}

		start_single_trajectory();
	}

	void topic_jointTrajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
	{
		if (msg->points.size() < 1)
		{
			RCLCPP_WARN(this->get_logger(),
						"Waypoint path requires at least 1 point, received %zu",
						msg->points.size());
			return;
		}

		// Validate joint count
		for (const auto &pt : msg->points)
		{
			if (pt.positions.size() != 6)
			{
				RCLCPP_ERROR(this->get_logger(),
							 "Each waypoint must have exactly 6 joint positions, got %zu",
							 pt.positions.size());
				return;
			}
		}

		if (!known_states_)
		{
			RCLCPP_WARN(this->get_logger(),
						"Cannot start waypoint path: joint states not yet received");
			return;
		}

		start_multi_trajectory(msg);
	}

	void start_single_trajectory()
	{
		std::lock_guard<std::mutex> lock(trajectory_mutex); // Lock Data

		// Calculate maximum error to determine trajectory duration
		double max_error = 0.0;
		for (int i = 0; i < 6; i++)
		{
			double error = std::abs(demanded_joints_[i] - current_joints_[i]);
			max_error = std::max(error, max_error);
		}

		// Set trajectory duration based on max error (use configurable max_velocity_)
		trajectory_duration_ = std::max(max_error / max_velocity_, 10.0); // Minimum 10 seconds

		// Calculate polynomial coefficients for each joint
		for (int i = 0; i < 6; i++)
		{
			// Mark joint as active only if it needs to move significantly
			double joint_error = std::abs(demanded_joints_[i] - current_joints_[i]);
			joint_active_[i] = (joint_error > 0.01); // 0.01 rad threshold (~0.5 degrees)

			coeffs_[i] = get_coeffs(
				current_joints_[i],
				demanded_joints_[i],
				current_velocities_[i],
				trajectory_duration_);

			// Reset PID Memory
			errors_integral_[i] = 0.0;
			errors_prev_[i] = 0.0;
		}

		// Record start time and set state
		trajectory_start_time_ = this->now();
		trajectory_state_ = TrajectoryState::SINGLE_SEGMENT;

		RCLCPP_INFO(this->get_logger(),
					"Starting single-segment trajectory, duration: %.2f seconds",
					trajectory_duration_);
	}

	void start_multi_trajectory(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
	{
		std::lock_guard<std::mutex> lock(trajectory_mutex);

		// Parse via points
		std::vector<ViaPoint> via_points;

		// Add current position as first via point
		ViaPoint start_pt;
		start_pt.positions.assign(current_joints_.begin(), current_joints_.end());
		start_pt.segment_duration = 0.0;
		start_pt.blend_time = 0.0;
		via_points.push_back(start_pt);

		// Add commanded via points
		for (const auto &pt : msg->points)
		{
			ViaPoint vp;
			vp.positions.assign(pt.positions.begin(), pt.positions.end());
			vp.segment_duration = 0.0; // Will be computed
			vp.blend_time = 0.0;	   // Will be computed
			via_points.push_back(vp);
		}

		// Compute trajectory parameters with jerk limiting
		compute_segment_durations(via_points);
		compute_blend_durations(via_points);
		adjust_blend_durations(via_points);

		// Build segments
		multi_trajectory_ = build_trajectory_segments(via_points);

		// Reset PID state
		for (int i = 0; i < 6; i++)
		{
			errors_integral_[i] = 0.0;
			errors_prev_[i] = 0.0;
		}

		// Start execution
		multi_trajectory_start_time_ = this->now();
		trajectory_state_ = TrajectoryState::MULTI_SEGMENT;

		RCLCPP_INFO(this->get_logger(),
					"Starting multi-segment trajectory with %zu via points, total duration: %.2fs",
					via_points.size(), multi_trajectory_.total_duration);
	}

	void send_msg(const std::vector<double> &vels)
	{
		auto msg = control_msgs::msg::JointJog();
		msg.header.frame_id = ""; // Initialize to prevent DDS serialization errors
		msg.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
		msg.velocities = vels;

		publisherJointPosition_->publish(msg);
	}

	void move_my_robot()
	{
		if (!known_states_)
		{
			return;
		}

		std::lock_guard<std::mutex> lock(trajectory_mutex);

		std::vector<double> velocities(6, 0.0);

		// State machine for trajectory execution
		switch (trajectory_state_)
		{
		case TrajectoryState::IDLE:
			// No active trajectory, send zero velocities
			send_msg(velocities);
			return;

		case TrajectoryState::SINGLE_SEGMENT:
			execute_single_trajectory(velocities);
			break;

		case TrajectoryState::MULTI_SEGMENT:
			execute_multi_trajectory(velocities);
			break;
		}

		send_msg(velocities);
	}

	void execute_single_trajectory(std::vector<double> &velocities)
	{
		double elapsed = (this->now() - trajectory_start_time_).seconds();

		// Check if trajectory is complete
		if (elapsed >= trajectory_duration_)
		{
			// Trajectory finished
			trajectory_state_ = TrajectoryState::IDLE;

			// Check if we're close enough to target
			double max_error = 0.0;
			for (int i = 0; i < 6; i++)
			{
				double error = std::abs(demanded_joints_[i] - current_joints_[i]);
				max_error = std::max(error, max_error);
			}

			if (max_error < 0.05)
			{
				RCLCPP_INFO(this->get_logger(), "Single-segment trajectory complete, at target");
			}
			else
			{
				RCLCPP_WARN(this->get_logger(),
							"Single-segment trajectory complete but error remains: %.4f rad",
							max_error);
			}
			return;
		}

		// Get desired position and velocity from single trajectory
		std::vector<double> desired_pos, desired_vel;
		evaluate_single_trajectory(elapsed, desired_pos, desired_vel);

		run_PID(desired_pos, desired_vel, velocities);
	}

	void execute_multi_trajectory(std::vector<double> &velocities)
	{
		double elapsed = (this->now() - multi_trajectory_start_time_).seconds();

		// Check if trajectory is complete
		if (elapsed >= multi_trajectory_.total_duration)
		{
			trajectory_state_ = TrajectoryState::IDLE;

			// Check if we're close enough to final target
			const ViaPoint &final_pt = multi_trajectory_.via_points.back();
			double max_error = 0.0;
			for (int i = 0; i < 6; i++)
			{
				double error = std::abs(final_pt.positions[i] - current_joints_[i]);
				max_error = std::max(error, max_error);
			}

			if (max_error < 0.05)
			{
				RCLCPP_INFO(this->get_logger(), "Multi-segment trajectory complete, at target");
			}
			else
			{
				RCLCPP_WARN(this->get_logger(),
							"Multi-segment trajectory complete but error remains: %.4f rad",
							max_error);
			}
			return;
		}

		// Get desired position and velocity from multi-segment trajectory
		std::vector<double> desired_pos, desired_vel;
		evaluate_multi_trajectory(elapsed, desired_pos, desired_vel);
		run_PID(desired_pos, desired_vel, velocities);
	}

	void run_PID(const std::vector<double> &desired_pos, const std::vector<double> &desired_vel, std::vector<double> &command_vel)
	{
		// Apply PID control with feedforward
		for (int i = 0; i < 6; i++)
		{
			// Only control joints that are actively moving in the trajectory
			if (!joint_active_[i])
			{
				command_vel[i] = 0.0; // Don't command inactive joints
				continue;
			}

			double error = desired_pos[i] - current_joints_[i];

			// Proportional Term
			double P = kp_ * error;

			// Integral Term
			errors_integral_[i] += error * dt_;
			errors_integral_[i] = std::clamp(errors_integral_[i], -integral_limit_, integral_limit_);
			double I = ki_ * errors_integral_[i];

			// Derivative Term
			double D = kd_ * (error - errors_prev_[i]) / dt_;

			// Feedforward Term + PID
			command_vel[i] = desired_vel[i] + P + I + D;

			// Clamp output velocity to prevent runaway
			command_vel[i] = std::clamp(command_vel[i], -max_velocity_ * 3.0, max_velocity_ * 3.0);

			errors_prev_[i] = error;
		}

		auto desired_joints_msg = sensor_msgs::msg::JointState();
		desired_joints_msg.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
		desired_joints_msg.position = desired_pos;
		desired_joints_msg.velocity = desired_vel;

		publisherDesiredJointStates_->publish(desired_joints_msg);
	}

	std::mutex trajectory_mutex;

	// PID Control parameters
	double kp_;
	double ki_;
	double kd_;
	double dt_;
	float integral_limit_;

	// Trajectory configuration (jerk limiting)
	double max_velocity_ = 0.25;	// rad/s
	double max_acceleration_ = 0.5; // rad/s^2
	double max_jerk_ = 1.0;			// rad/s^3 (configurable)
	double min_blend_time_ = 0.1;	// seconds
	double min_segment_time_ = 0.5; // seconds

	// State flags
	bool known_states_ = false;

	// State machine for trajectory execution
	TrajectoryState trajectory_state_ = TrajectoryState::IDLE;

	// Robot state
	std::vector<float> current_joints_;
	std::vector<float> current_velocities_;
	std::vector<float> demanded_joints_;

	// Single-segment (legacy) trajectory data
	std::vector<PolynomialCoeffs> coeffs_;
	rclcpp::Time trajectory_start_time_;
	double trajectory_duration_;
	std::vector<bool> joint_active_; // Track which joints should move

	// Multi-segment trajectory data
	MultiSegmentTrajectory multi_trajectory_;
	rclcpp::Time multi_trajectory_start_time_;

	// PID memory
	std::vector<float> errors_prev_;
	std::vector<float> errors_integral_;

	// ROS infrastructure
	rclcpp::CallbackGroup::SharedPtr cb_group_;
	rclcpp::CallbackGroup::SharedPtr tm_group_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisherJointPosition_;
	rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisherDesiredJointStates_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscriptionJointPosition_;
	rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriptionJointDemands_;
	rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr subscriptionJointTrajectory_;
	size_t count_;
};

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<GoalMovementMover6>());
	rclcpp::shutdown();
	return 0;
}
