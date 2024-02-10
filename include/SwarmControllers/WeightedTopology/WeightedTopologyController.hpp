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
    using WeightedTopologyNeighbors = custom_msgs::msg::WeightedTopologyNeighbors;
    using SwarmControllerType = SwarmController<custom_msgs::msg::WeightedTopologyNeighbors>;
    using TrajectorySetpoint = SwarmControllerType::TrajectorySetpoint;
    using PoseTwist = Eigen::Vector<float, 6>;
    using Neighborhood = Eigen::Matrix<float, 6, Eigen::Dynamic>;
    using Gains = Eigen::Vector<float, 6>;
public:
    WeightedTopologyController();

private:
    /**
     *
     * @param neighbors
     */
    void neighbors_to_matrix(const WeightedTopologyNeighbors &neighbors);

    /**
     *
     * @param neighbors
     */
    void neighbors_callback(const WeightedTopologyNeighbors::SharedPtr &neighbors) override;

    /**
     *
     */
    void timer_callback() override;

    /**
     *
     */
    void compute_command(TrajectorySetpoint& setpoint);

private:
    Neighborhood neighborhood;
    Gains gains;
    bool is_neighborhood_empty{true};
    // TODO: Take it as a parameter
    std::vector<float> default_pose{0., 0., 3.};
};