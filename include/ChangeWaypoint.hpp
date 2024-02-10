/**
 * @file change_waypoint_node.cpp
 * @brief ROS 2 Node for changing waypoints based on vehicle position.
 * @author Arthur Astier
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <yaml-cpp/yaml.h>

/**
 * @class ChangeWaypoint
 * @brief ROS 2 Node for changing waypoints based on vehicle position.
 */
class ChangeWaypoint : public rclcpp::Node {
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;

public:
    /**
     * @brief Constructor for the ChangeWaypoint class.
     */
    ChangeWaypoint();

private:
    /**
     * @brief Publish the offboard control mode.
     */
    void publish_offboard_control_mode();

    /**
     * @brief Callback function for the vehicle local position subscriber.
     * @param pose The received vehicle local position message.
     */
    void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose);

    /**
     * @brief Get the value of a coordinate from the waypoints.
     * @param idx Index of the waypoint.
     * @param var Variable for the coordinate (e.g., "x", "y", "z", "yaw").
     * @return The coordinate value.
     */
    double coord(const size_t idx, const std::string &var);

    /**
     * @brief Write the waypoint to be published based on the specified index.
     * @param idx Index of the waypoint.
     */
    void writeWP(const std::size_t idx);

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pose_subscriber;

    TrajectorySetpoint waypoint;
    std::size_t wp_idx{0u};
    YAML::Node waypoints;
    double threshold{};
    double threshold_angle{};

    double x_init{};
    double y_init{};
};