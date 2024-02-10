/**
 * @file arming_node.cpp
 * @brief ROS 2 Node for arming multiple drones using PX4 Vehicle Command Service.
 * @author Arthur Astier
 */

#pragma once

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

/**
 * @class Arming
 * @brief ROS 2 Node for arming multiple drones.
 */
class Arming : public rclcpp::Node {
    using ArmCheckClient = rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr;
    using VehicleCommand = px4_msgs::msg::VehicleCommand;

public:
    /**
     * @brief Constructor for the Arming class.
     */
    Arming();

    /**
     * @brief Arm the specified drone.
     * @param drone_idx Index of the drone to be armed.
     */
    void arm(std::size_t drone_idx);

    /**
     * @brief Set the specified drone to offboard mode.
     * @param drone_idx Index of the drone to be set to offboard mode.
     */
    void offboard_mode(std::size_t drone_idx);

private:
    /**
     * @brief Request a vehicle command for the specified drone.
     * @param command Vehicle command to be requested.
     * @param drone_idx Index of the drone.
     * @param param1 Command parameter 1.
     * @param param2 Command parameter 2.
     */
    void request_vehicle_command(uint16_t command, std::size_t drone_idx, float param1 = 0.0, float param2 = 0.0);

private:
    std::vector<ArmCheckClient> arm_check_clients_;
    std::vector<bool> is_armed;
    std::vector<bool> is_offboard;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    std::size_t nb_drones;
};