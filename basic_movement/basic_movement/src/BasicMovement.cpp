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
#include <control_msgs/msg/joint_jog.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sstream>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class BasicMovement : public rclcpp::Node
{
public:

  BasicMovement()
  : Node("BasicMovement"), count_(0)
  {
    publisher_ = this->create_publisher<control_msgs::msg::JointJog>("/JointJog", 10);
    timer_ = this->create_wall_timer(
      4000ms, std::bind(&BasicMovement::timer_callback, this));
  }

private:

  void send_msg(float vel) {
	   auto msg_start = control_msgs::msg::JointJog();
	  //message.data = "Hello, world! " + std::to_string(count_++);
    
      std::stringstream ss;
	  ss << "joint1";
      msg_start.joint_names.push_back(ss.str());
	  msg_start.velocities.push_back(vel);
    
      publisher_->publish(msg_start);
  }
  void timer_callback()
  {
	if(state==0) {
	  send_msg(0.5);
      state=1;
	}
	else if(state==1) {
	  send_msg(0);
      state=2;	
	}
	else if(state==2) {
	  send_msg(-0.5);
      state=3;	
	}
	else if(state==3) {
	  send_msg(0);
      state=0;	
	}
   
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr publisher_;  
  unsigned long long state=0;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BasicMovement>());
  rclcpp::shutdown();
  return 0;
}
