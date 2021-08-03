#include <stdio.h>
#include <string.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "unistd.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class VirtualStageNode : public rclcpp::Node
{
public:
    using Float64 = std_msgs::msg::Float64;
    using JointTrajectory = trajectory_msgs::msg::JointTrajectory;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    explicit VirtualStageNode() : Node("virtual_stage_node")
    {

        RCLCPP_INFO(this->get_logger(), "Initializing virtual stage...");
        // Start stage pose publisher
        x_publisher = this->create_publisher<Float64>("virtual_stage/joint_states/x", 10);
        z_publisher = this->create_publisher<Float64>("virtual_stage/joint_states/z", 10);

        timer = this->create_wall_timer(
            50ms, std::bind(&VirtualStageNode::publish_state, this));

        // Start virtual stage position command subscribers
        x_subscriber = this->create_subscription<Float64>(
            "virtual_stage/x_position_controller/command",
            10,
            std::bind(&VirtualStageNode::x_command_callback, this, std::placeholders::_1));

        z_subscriber = this->create_subscription<Float64>(
            "virtual_stage/z_position_controller/command",
            10,
            std::bind(&VirtualStageNode::z_command_callback, this, std::placeholders::_1));

        velocity_subscriber = this->create_subscription<Float64>(
            "virtual_stage/velocity_controller",
            10,
            std::bind(&VirtualStageNode::velocity_callback, this, std::placeholders::_1));

        //Start joint state subscriber
        joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&VirtualStageNode::topic_callback, this, std::placeholders::_1));

        joint1_client = rclcpp_action::create_client<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/joint1_trajectory_controller/follow_joint_trajectory");

        joint2_client = rclcpp_action::create_client<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/joint2_trajectory_controller/follow_joint_trajectory");

        joint3_client = rclcpp_action::create_client<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/joint3_trajectory_controller/follow_joint_trajectory");

        joint4_client = rclcpp_action::create_client<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/joint4_trajectory_controller/follow_joint_trajectory");
        
        RCLCPP_INFO(this->get_logger(), "Ready.");
    }

private:
    rclcpp::Publisher<Float64>::SharedPtr x_publisher;
    rclcpp::Publisher<Float64>::SharedPtr z_publisher;
    rclcpp::Subscription<Float64>::SharedPtr x_subscriber;
    rclcpp::Subscription<Float64>::SharedPtr z_subscriber;
    rclcpp::Subscription<Float64>::SharedPtr velocity_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint1_client;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint2_client;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint3_client;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr joint4_client;

    double current_x;
    double current_z;
    double target_x;
    double target_z;
    double target_velocity = 0.0125;
    double target_time;

    void x_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_x = msg->data;

        double time = abs(target_x - current_x) / target_velocity;
        MoveJoint(joint1_client, "joint1", target_x, time);
    }

    void z_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_z = msg->data;

        double time = abs(target_z - current_z) / target_velocity;
        MoveJoint(joint2_client, "joint2", target_z, time);
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        auto names = msg->name;
        int index1, index2;
        for (int i = 0; i < names.size(); ++i)
        {
            std::string name = names[i];
            if (name.find("joint1") != std::string::npos)
            {
                index1 = i;
            }
            else if (name.find("joint2") != std::string::npos)
            {
                index2 = i;
            }
        }

        current_x = msg->position[index1];
        current_z = msg->position[index2];
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_velocity = msg->data;
    }

    void publish_state()
    {
        // Publish state
        auto x = std_msgs::msg::Float64();
        auto z = std_msgs::msg::Float64();

        x.data = current_x;
        z.data = current_z;

        x_publisher->publish(x);
        z_publisher->publish(z);
    }

    void MoveJoint(rclcpp_action::Client<FollowJointTrajectory>::SharedPtr client, std::string joint, double position, double time)
    {
        // Initialize JointTrajectoryPoint message
        std::vector<std::string> joint_names = {joint};
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

        //set x and z
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration::from_seconds(time);
        point.positions.resize(joint_names.size());

        point.positions[0] = position;

        points.push_back(point);

        // Send message to action server
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
        control_msgs::action::FollowJointTrajectory_Goal goal_msg;
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(5.0);
        goal_msg.trajectory.joint_names = joint_names;
        goal_msg.trajectory.points = points;

        auto goal_handle_future = client->async_send_goal(goal_msg, opt);
    }

}; // class StageVirtualNode

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualStageNode>());
    rclcpp::shutdown();
    return 0;
}
