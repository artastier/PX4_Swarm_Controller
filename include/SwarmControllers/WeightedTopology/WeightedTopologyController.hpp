/**
 * @brief  This controller is based on the paper:
 *
 * Distributed leader-follower formation control for multiple quadrotors with weighted topology,
 *
 * Zhicheng Hou, Isabelle Fantoni, hal-01180491
 *
 * @author Arthur Astier
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <eigen3/Eigen/Eigen>
#include "SwarmController.hpp"
#include <custom_msgs/msg/weighted_topology_neighbors.hpp>


// TODO: Add doxygen
class WeightedTopologyController : public SwarmController<custom_msgs::msg::WeightedTopologyNeighbors> {
    using OffboardControlMode = SwarmController::OffboardControlMode;
    using WeightedTopologyNeighbors = custom_msgs::msg::WeightedTopologyNeighbors;
public:
    WeightedTopologyController();

private:
    void publish_offboard_control_mode() override;

    void neighbors_callback(const WeightedTopologyNeighbors::SharedPtr &neighbors) override;

    void timer_callback() override;

private:

};