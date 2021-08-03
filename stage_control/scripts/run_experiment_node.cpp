#include <chrono>
#include <functional>
#include <memory>
#include <thread>
#include <iostream>
#include <fstream>

#include "stage_control_interfaces/action/move_stage.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;


namespace stage_control
{
    class RunExperimentNode : public rclcpp::Node
    {
    public:
        using MoveStage = stage_control_interfaces::action::MoveStage;
        using GoalHandleMoveStage = rclcpp_action::ClientGoalHandle<MoveStage>;
        using Float64 = std_msgs::msg::Float64;

        //STAGE_CONTROL_PUBLIC
        explicit RunExperimentNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
            : Node("run_experiment_node", options)
        {
            using namespace std::placeholders;

            // Get velocity
            this->declare_parameter<double>("velocity", 0.05);
            this->get_parameter("velocity", velocity);
            RCLCPP_INFO(this->get_logger(), "Starting experiment using velocity: %f.", velocity);

            // Get data filepath
            this->declare_parameter<std::string>("filepath", "test.txt");
            this->get_parameter("filepath", filepath);

            // Subscribe to joint state topics
            hardware_x_subscriber = this->create_subscription<Float64>(
                "stage/joint_states/x",
                10,
                std::bind(&RunExperimentNode::hardware_x_callback, this, std::placeholders::_1));

            hardware_z_subscriber = this->create_subscription<Float64>(
                "stage/joint_states/z",
                10,
                std::bind(&RunExperimentNode::hardware_z_callback, this, std::placeholders::_1));

            virtual_x_subscriber = this->create_subscription<Float64>(
                "virtual_stage/joint_states/x",
                10,
                std::bind(&RunExperimentNode::virtual_x_callback, this, std::placeholders::_1));

            virtual_z_subscriber = this->create_subscription<Float64>(
                "virtual_stage/joint_states/z",
                10,
                std::bind(&RunExperimentNode::virtual_z_callback, this, std::placeholders::_1));

            // Publish to velocity topic
            velocity_publisher = this->create_publisher<Float64>("virtual_stage/velocity_controller", 10);

            // Start action client
            this->action_client_ = rclcpp_action::create_client<MoveStage>(
                this,
                "move_stage");

            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(40),
                std::bind(&RunExperimentNode::calculate_error, this));

            RCLCPP_INFO(this->get_logger(), "Running experiment...");
            this->Run();
        }

        void Run()
        {
            // Set velocity
            Float64 msg;
            msg.data = velocity;
            velocity_publisher->publish(msg);

            rclcpp::Rate delay(2.0);
            delay.sleep();

            // Open file
            file.open(filepath);

            is_running = true;
            
            // Get initial time
            start_time = this->now().seconds();

            RCLCPP_INFO(this->get_logger(), "Moving to initial position...");
            stage = 0;
            Move(0, 0);
        }

    private:
        rclcpp::Subscription<Float64>::SharedPtr hardware_x_subscriber;
        rclcpp::Subscription<Float64>::SharedPtr hardware_z_subscriber;
        rclcpp::Subscription<Float64>::SharedPtr virtual_x_subscriber;
        rclcpp::Subscription<Float64>::SharedPtr virtual_z_subscriber;
        rclcpp::Publisher<Float64>::SharedPtr velocity_publisher;
        rclcpp_action::Client<MoveStage>::SharedPtr action_client_;
        rclcpp::TimerBase::SharedPtr timer_;
        std::string filepath;
        double hardware_x;
        double hardware_z;
        double virtual_x;
        double virtual_z;
        double velocity;
        double start_time;
        bool is_running = false;
        int stage = 0;
        char buffer[100];
        std::ofstream file;

        void hardware_x_callback(const Float64::SharedPtr msg)
        {
            this->hardware_x = msg->data;
        }

        void hardware_z_callback(const Float64::SharedPtr msg)
        {
            this->hardware_z = msg->data;
        }

        void virtual_x_callback(const Float64::SharedPtr msg)
        {
            this->virtual_x = msg->data;
        }

        void virtual_z_callback(const Float64::SharedPtr msg)
        {
            this->virtual_z = msg->data;
        }

        void calculate_error()
        {
            //RCLCPP_INFO(this->get_logger(), "Passing...");
            if (!is_running)
                return;
            double dx = hardware_x - virtual_x;
            double dz = hardware_z - virtual_z;
            double elapsed = this->now().seconds() - this->start_time;
            RCLCPP_INFO(this->get_logger(), "Time: %f, Error: %f, %f", elapsed, dx, dz);

            snprintf( buffer, sizeof( buffer ), "%f, %f, %f, %f, %f, %f, %f\n", 
                elapsed, hardware_x, hardware_z, virtual_x, virtual_z, dx, dz);
            file << buffer;
        }

        void Move(double x, double z)
        {
            auto send_goal_options = rclcpp_action::Client<MoveStage>::SendGoalOptions();
            send_goal_options.result_callback =
                std::bind(&RunExperimentNode::result_callback, this, _1);

            action_client_->wait_for_action_server();

            MoveStage::Goal goal;
            goal.set__x(x);
            goal.set__z(z);
            goal.set__eps(0.001);

            action_client_->async_send_goal(goal, send_goal_options);
        }

        void result_callback(const GoalHandleMoveStage::WrappedResult &result)
        {
            rclcpp::Rate rate(2);
            rate.sleep();
            if (stage == 0)
            {
                RCLCPP_INFO(this->get_logger(), "Moving to first position...");
                Move(0.1, 0);
                stage = 1;
            }
            else if (stage == 1)
            {
                RCLCPP_INFO(this->get_logger(), "Moving to second position...");
                Move(0.1, 0.1);
                stage = 2;
            }
            else if (stage == 2)
            {
                RCLCPP_INFO(this->get_logger(), "Moving to third position...");
                Move(0, 0);
                stage = 3;
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "Experiment complete.");
                file.close();
                rclcpp::shutdown();;
            }
        }
    }; // class RunExperimentNode

} // namespace stage_control

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto experiment_node = std::make_shared<stage_control::RunExperimentNode>();
    rclcpp::spin(experiment_node);

    rclcpp::shutdown();
    return 0;
}
