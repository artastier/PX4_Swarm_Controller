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

enum class CONTROL{
    POSITION = 0,
    VELOCITY = 1,
    ACCELERATION = 2,
    ATTITUDE = 3,
    BODY_RATE = 4,
    THRUST_TORQUE = 5,
    DIRECT_ACTUATOR = 6
};

// TODO: Add doxygen
template<typename Neighbors>
class SwarmController : public rclcpp::Node {
    // This trait ensure checking std::empty(neighborhood.neighbors_position) won't crash
    static_assert(traits::has_neighbors_position_attribute_and_is_VLP_v<Neighbors>,
                  "Neighbors type must have neighbors_position attribute of type vector<px4_msgs::msg::VehicleLocalPosition>");
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

protected:
    void publish_offboard_control_mode(const CONTROL& control){
        OffboardControlMode msg{};
        switch (control) {
            case CONTROL::POSITION:
                msg.position = true;
                break;
            case CONTROL::VELOCITY:
                msg.velocity = true;
                break;
            case CONTROL::ACCELERATION:
                msg.acceleration = true;
                break;
            case CONTROL::ATTITUDE:
                msg.attitude = true;
                break;
            case CONTROL::BODY_RATE:
                msg.body_rate = true;
                break;
            case CONTROL::THRUST_TORQUE:
                msg.thrust_and_torque = true;
                break;
            case CONTROL::DIRECT_ACTUATOR:
                msg.thrust_and_torque = true;
                break;
        }
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }
private:
    virtual void neighbors_callback(const typename Neighbors::SharedPtr &neighbors) = 0;

    virtual void timer_callback() = 0;

protected:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    typename rclcpp::Subscription<Neighbors>::SharedPtr neighbors_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

};


