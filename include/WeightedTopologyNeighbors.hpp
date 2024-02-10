/**
 * @brief
 * @author Astier Arthur
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "NearestNeighbors.hpp"
#include <custom_msgs/msg/weighted_topology_neighbors.hpp>
#include <eigen3/Eigen/Eigen>

// TODO: Generate Doxygen
class WeightedTopologyNeighbors : public NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors> {
    using WeightedTopologyNeighborsMsg = custom_msgs::msg::WeightedTopologyNeighbors;
    using PRCS = Eigen::Vector<std::size_t, Eigen::Dynamic>;
    using Weights = std::vector<double>;

public:
    WeightedTopologyNeighbors();

private:
    void process_neighbor_position(const std::size_t neighbor_idx, const VehicleLocalPosition &position,
                                   const VehicleLocalPosition &neighbor_position,
                                   WeightedTopologyNeighborsMsg &neighborhood) override;

    void process_neighborhood(const std::size_t drone_idx, WeightedTopologyNeighborsMsg &neighborhood) override;

    void enrich_neighborhood(WeightedTopologyNeighborsMsg &neighborhood) override;

private:
    std::vector<bool> leaders;
    PRCS prcs;
    PRCS prcs_neighborhood;
};