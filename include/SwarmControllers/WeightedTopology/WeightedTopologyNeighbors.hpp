/**
 * @brief WeightedTopologyNeighbors class for calculating weighted topology neighbors based on vehicle positions.
 * @details This class extends the NearestNeighbors class template to calculate weighted topology neighbors.
 * It provides functionality to process neighbor positions, enrich neighborhoods, and handle weighted topology neighbors.
 * @author Astier Arthur
*/
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "NearestNeighbors.hpp"
#include <custom_msgs/msg/weighted_topology_neighbors.hpp>
#include <eigen3/Eigen/Eigen>

namespace Neighborhood {
    class WeightedTopologyNeighbors : public NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors> {
        using WeightedTopologyNeighborsMsg = custom_msgs::msg::WeightedTopologyNeighbors;
        using PRCS = Eigen::Vector<std::size_t, Eigen::Dynamic>;
        using Weights = std::vector<double>;
        using Vector3d = Eigen::Vector3d;
        using Formation = std::vector<Vector3d>;

    public:
        /**
         * @brief Constructor for WeightedTopologyNeighbors class.
         */
        WeightedTopologyNeighbors();

    private:
        /**
         * @brief Converts vectors representing x, y, and z coordinates to Eigen Vector3d objects.
         * @param x The x-coordinates vector.
         * @param y The y-coordinates vector.
         * @param z The z-coordinates vector.
         */
        void
        vectors_to_Vector3d(const std::vector<double> &x, const std::vector<double> &y, const std::vector<double> &z);

        /**
         * @brief Processes the position of a neighbor drone.
         * @param drone_idx The index of the current drone.
         * @param neighbor_idx The index of the neighbor drone.
         * @param position The position of the current drone.
         * @param neighbor_position The position of the neighbor drone.
         * @param neighborhood The neighborhood message to be updated.
         */
        void process_neighbor_position(const std::size_t drone_idx, const std::size_t neighbor_idx,
                                       const VehicleLocalPosition &position,
                                       VehicleLocalPosition neighbor_position,
                                       WeightedTopologyNeighborsMsg &neighborhood) override;

        /**
         * @brief Processes the neighborhood based on the positions of the neighbor drones.
         * @param drone_idx The index of the current drone.
         * @param neighborhood The neighborhood message to be updated.
         */
        void process_neighborhood(const std::size_t drone_idx, WeightedTopologyNeighborsMsg &neighborhood) override;

        /**
         * @brief Enriches the neighborhood message with weights calculated based on the PRCS (priority based on conflict state) of each neighbor.
         * @param neighborhood The neighborhood message to be enriched.
         */
        void enrich_neighborhood(WeightedTopologyNeighborsMsg &neighborhood) override;

    private:
        std::vector<bool> leaders; ///< Vector indicating whether each drone is a leader.
        PRCS prcs; ///< Priority Coefficient (PrC) for each drone.
        PRCS prcs_neighborhood; ///< Temporary vector to update the PrC of one drone (per vector in the paper).
        Formation formation; ///< Pose of each drone in the swarm.
    };
}