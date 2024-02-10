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

// TODO: Add doxygen
class WeightedTopologyController : public SwarmController {
    using OffboardControlMode = SwarmController::OffboardControlMode;
    using Neighbors = SwarmController::Neighbors;
    using PRCS = Eigen::Vector<std::size_t, Eigen::Dynamic>;
    using Weights = Eigen::VectorXd;
public:
    WeightedTopologyController();

private:
    void publish_offboard_control_mode() override;

    void neighbors_callback(const Neighbors::SharedPtr &neighbors) override;

    void timer_callback() override;

private:
    PRCS prcs;
    Weights weights;

    std::size_t drone_id{};
    std::size_t nb_drones{};

    double x_init{};
    double y_init{};

};