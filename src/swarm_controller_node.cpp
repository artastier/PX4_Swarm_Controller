/**
* @author Arthur Astier
*/

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <custom_msgs/msg/neighbors.hpp>

using namespace std::chrono_literals;

class SwarmController : public rclcpp::Node {
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
public:
    using Neighbors = custom_msgs::msg::Neighbors;
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;

public:
    SwarmController();

private:
    virtual void publish_offboard_control_mode() = 0;

    virtual void neighbors_callback(const Neighbors::SharedPtr &neighbors) = 0;

    virtual void timer_callback() = 0;

protected:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<Neighbors>::SharedPtr neighbors_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

};

SwarmController::SwarmController() : rclcpp::Node("swarm_controller") {
    const std::string name_space{this->get_namespace()};

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
            name_space + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
            name_space + "/fmu/in/trajectory_setpoint", 10);

    neighbors_subscriber = this->create_subscription<Neighbors>(name_space + "/fmu/out/nearest_neighbors", 10,
                                                                [this](const Neighbors::SharedPtr neighbors) {
                                                                    neighbors_callback(neighbors);
                                                                });

    timer = this->create_wall_timer(100ms, [this](){timer_callback();});
}

