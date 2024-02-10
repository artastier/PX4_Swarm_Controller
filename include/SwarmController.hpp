/**
* @author Arthur Astier
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "NeighborsTraits.hpp"
#include <chrono>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;

// TODO: Add doxygen
template<typename Neighbors>
class SwarmController : public rclcpp::Node {
    static_assert(traits::has_shared_ptr_v<Neighbors>,"The Neighbors type doesn't define ::SharedPtr");
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
protected:
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;

public:
    SwarmController() : rclcpp::Node("swarm_controller") {
        const std::string name_space{this->get_namespace()};

        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
                name_space + "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
                name_space + "/fmu/in/trajectory_setpoint", 10);

        neighbors_subscriber = this->create_subscription<Neighbors>(name_space + "/fmu/out/nearest_neighbors", 10,
                                                                    [this](const typename Neighbors::SharedPtr neighbors) {
                                                                        neighbors_callback(neighbors);
                                                                    });

        timer = this->create_wall_timer(100ms, [this]() { timer_callback(); });
    }

private:
    virtual void publish_offboard_control_mode() = 0;

    virtual void neighbors_callback(const typename Neighbors::SharedPtr &neighbors) = 0;

    virtual void timer_callback() = 0;

protected:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    typename rclcpp::Subscription<Neighbors>::SharedPtr neighbors_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

};


