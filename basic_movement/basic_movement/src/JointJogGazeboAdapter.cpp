// JointJogGazeboAdapter.cpp
// Converts JointJog messages to Float64MultiArray for velocity_controller

#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

class JointJogGazeboAdapter : public rclcpp::Node
{
public:
    JointJogGazeboAdapter()
        : Node("joint_jog_gazebo_adapter")
    {
        joint_names_ = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        velocity_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/velocity_controller/commands", 10);

        joint_jog_sub_ = this->create_subscription<control_msgs::msg::JointJog>(
            "/JointJog", 10,
            std::bind(&JointJogGazeboAdapter::joint_jog_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "JointJogGazeboAdapter: /JointJog -> /velocity_controller/commands");
    }

private:
    void joint_jog_callback(const control_msgs::msg::JointJog::SharedPtr msg)
    {
        auto cmd_msg = std_msgs::msg::Float64MultiArray();
        cmd_msg.data.resize(joint_names_.size(), 0.0);

        for (size_t i = 0; i < msg->joint_names.size() && i < msg->velocities.size(); ++i)
        {
            for (size_t j = 0; j < joint_names_.size(); ++j)
            {
                if (msg->joint_names[i] == joint_names_[j])
                {
                    cmd_msg.data[j] = msg->velocities[i];
                    break;
                }
            }
        }
        velocity_pub_->publish(cmd_msg);
    }

    std::vector<std::string> joint_names_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr velocity_pub_;
    rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_jog_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointJogGazeboAdapter>());
    rclcpp::shutdown();
    return 0;
}
