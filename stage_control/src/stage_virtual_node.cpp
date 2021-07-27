#include <stdio.h>
#include <string.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <functional>
#include <memory>
#include <thread>

#include "unistd.h"
#include "stage_control_interfaces/action/move_stage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

using namespace std::chrono_literals;

class VirtualStageNode : public rclcpp::Node
{
public:
    using Float64 = std_msgs::msg::Float64;

    explicit VirtualStageNode() : Node("virtual_stage_node")
        {
            RCLCPP_INFO(this->get_logger(), "Initializing virtual stage...");
            target_x = current_x;
            target_z = current_z;

            timer = this->create_wall_timer(
                50ms, std::bind(&VirtualStageNode::publish_state, this));

            // Start virtual stage position command subscribers
            x_subscriber = this->create_subscription<std_msgs::msg::Float64>(
                "virtual_stage/x_position_controller/x",
                10,
                std::bind(&VirtualStageNode::x_command_callback, this, std::placeholders::_1));

            z_subscriber = this->create_subscription<std_msgs::msg::Float64>(
                "virtual_stage/x_position_controller/z",
                10,
                std::bind(&VirtualStageNode::z_command_callback, this, std::placeholders::_1));

            //run();
            RCLCPP_INFO(this->get_logger(), "Ready.");
        }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr x_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr z_subscriber;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr virtual_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    double current_x;
    double current_z;
    double target_x;
    double target_z;

    void x_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_x = msg->data;
    }

    void z_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_z = msg->data;
    }

    void publish_state()
    {
        // Emulate motion
        current_x += (target_x - current_x) / 10;
        current_z += (target_z - current_z) / 10;
        MoveVirtualStage(current_x, current_z);

        // Publish state
        auto x = std_msgs::msg::Float64();
        auto z = std_msgs::msg::Float64();

        x.data = current_x;
        z.data = current_z;
    }
    
    void MoveVirtualStage(double x, double z)
        {
         // Initialize JointTrajectoryPoint message
            std::vector<std::string> joint_names = {"joint1","joint2"};
            std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

            // Set x and z position
            auto x_position = Float64();
            auto z_position = Float64();

            x_position.data = x;
            z_position.data = z;
            
            //set z
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.time_from_start = rclcpp::Duration::from_seconds(1.0);  // start asap
            point.positions.resize(joint_names.size());

            point.positions[0] = x;
            point.positions[1] = z;

            points.push_back(point);

            // Create FollowJointTrajectory message
            
            control_msgs::action::FollowJointTrajectory_Goal goal_msg;
            trajectory_msgs::msg::JointTrajectory trajectory;
            trajectory.joint_names = joint_names;
            trajectory.points = points;
        
            // Send to JointTrajectoryController server
            //auto goal_handle_future = rcl_action_client_t->async_send_goal(goal_msg, opt);
            this->virtual_publisher->publish(trajectory);

        }

}; // class StageVirtualNode

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualStageNode>());
    rclcpp::shutdown();
    return 0;
}