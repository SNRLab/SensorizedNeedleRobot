#include <stdio.h>
#include <string.h>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class EmulatedStage : public rclcpp::Node
{
public:
    explicit EmulatedStage() : Node("stage_emulated_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing emulated stage...");
        target_x = current_x;
        target_z = current_z;

        // Start stage pose publisher
        x_publisher = this->create_publisher<std_msgs::msg::Float64>("emulated_stage/joint_states/x", 10);
        z_publisher = this->create_publisher<std_msgs::msg::Float64>("emulated_stage/joint_states/z", 10);
        timer = this->create_wall_timer(
            50ms, std::bind(&EmulatedStage::publish_state, this));

        // Start stage position command subscribers
        x_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "emulated_stage/x_position_controller/command",
            10,
            std::bind(&EmulatedStage::x_command_callback, this, std::placeholders::_1));

        z_subscriber = this->create_subscription<std_msgs::msg::Float64>(
            "emulated_stage/z_position_controller/command",
            10,
            std::bind(&EmulatedStage::z_command_callback, this, std::placeholders::_1));

        //run();
        RCLCPP_INFO(this->get_logger(), "Ready.");
    }

    ~EmulatedStage()
    {
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr x_publisher;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr z_publisher;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr x_subscriber;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr z_subscriber;
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

        // Publish state
        auto x = std_msgs::msg::Float64();
        auto z = std_msgs::msg::Float64();

        x.data = current_x;
        z.data = current_z;

        // Publish
        x_publisher->publish(x);
        z_publisher->publish(z);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EmulatedStage>());
    rclcpp::shutdown();
    return 0;
}