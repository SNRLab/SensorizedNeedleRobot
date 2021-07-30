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

    explicit VirtualStageNode() : Node("virtual_stage_node")
    {
        
        RCLCPP_INFO(this->get_logger(), "Initializing virtual stage...");
        // Start stage pose publisher
        x_publisher = this->create_publisher<std_msgs::msg::Float64>("virtual_stage/joint_states/x", 10);
        z_publisher = this->create_publisher<std_msgs::msg::Float64>("virtual_stage/joint_states/z", 10);
        virtual_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("joint_trajectory_controller/joint_trajectory", 10);
        
        timer = this->create_wall_timer(
            50ms, std::bind(&VirtualStageNode::publish_state, this));

        // Start virtual stage position command subscribers
        x_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "virtual_stage/x_position_controller/x",
            10,
            std::bind(&VirtualStageNode::x_command_callback, this, std::placeholders::_1));

        z_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "virtual_stage/z_position_controller/z",
            10,
            std::bind(&VirtualStageNode::z_command_callback, this, std::placeholders::_1));

        velocity_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "virtual_stage/velocity_controller",
            10,
            std::bind(&VirtualStageNode::velocity_callback, this, std::placeholders::_1));

        //Start joint state subscriber
        joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states",
            10,
            std::bind(&VirtualStageNode::topic_callback, this, std::placeholders::_1));

        action_client = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_graph_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/joint_trajectory_controller/follow_joint_trajectory");
        //run();
        RCLCPP_INFO(this->get_logger(), "Ready.");
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr x_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr z_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_subscriber;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr virtual_publisher;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client;


    double current_x;
    double current_z;
    double target_x;
    double target_z;
    double target_velocity;
    double target_time;

    void x_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_x = msg->data;
    }

    void z_command_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_z = msg->data;
    }

    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_x = msg->position[0];
        current_z = msg->position[1];
    }

    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg)
    {
        target_velocity = msg->data;
    }


    void publish_state()
    {
        // Emulate motion
        MoveVirtualStage(target_x, target_z);

        // Publish state
        auto x = std_msgs::msg::Float64();
        auto z = std_msgs::msg::Float64();

        x.data = current_x;
        z.data = current_z;

        x_publisher->publish(x);
        z_publisher->publish(z);
    }

    double determine_time(double x, double z, double velocity)
    {

        //find distance
        double x2 = pow(x, 2);
        double z2 = pow(z, 2);
        double dist = sqrt(x2+ z2);

        double time = dist/velocity;
        return time;

    }

    void MoveVirtualStage(double x, double z)
    {
        
        // Initialize JointTrajectoryPoint message
        std::vector<std::string> joint_names = {"joint1", "joint2"};
        std::vector<trajectory_msgs::msg::JointTrajectoryPoint> points;

        //set x and z
        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.time_from_start = rclcpp::Duration::from_seconds(determine_time(target_x,target_z,target_velocity)); 
        point.positions.resize(joint_names.size());

        point.positions[0] = x;
        point.positions[1] = z;

        points.push_back(point);
        
        // Send message to action server
        rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions opt;
        control_msgs::action::FollowJointTrajectory_Goal goal_msg;
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(5.0);
        goal_msg.trajectory.joint_names = joint_names;
        goal_msg.trajectory.points = points;

        auto goal_handle_future = action_client->async_send_goal(goal_msg, opt);
        
    }

}; // class StageVirtualNode

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VirtualStageNode>());
    rclcpp::shutdown();
    return 0;
}
