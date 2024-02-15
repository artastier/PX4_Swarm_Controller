/**
 * @brief NearestNeighbors class template for finding nearest neighbors based on vehicle positions.
 * @details This class template provides functionality to find nearest neighbors among a group of vehicles based on their positions.
 * @tparam Neighbors Type representing the neighbors.
 * @author Arthur Astier
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "NeighborsTraits.hpp"
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <eigen3/Eigen/Eigen>

using namespace std::chrono_literals;
namespace Neighborhood {
    template<typename Neighbors>
    class NearestNeighbors : public rclcpp::Node {
        // This trait ensure checking std::empty(neighborhood.neighbors_position) won't crash
        static_assert(traits::has_neighbors_position_attribute_and_is_VLP_v<Neighbors>,
                      "Neighbors type must have neighbors_position attribute of type vector<px4_msgs::msg::VehicleLocalPosition>");

    protected:
        using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;

    private:
        using Vector2d = Eigen::Vector2d;
        // TODO: Refactor x_init and y_init using PoseInit
        using PoseInit = std::vector<Vector2d>;
        using PositionSubscriberSharedPtr = rclcpp::Subscription<VehicleLocalPosition>::SharedPtr;
        using NeighborsPublisherSharedPtr = typename rclcpp::Publisher<Neighbors>::SharedPtr;

    public:
        /**
         * @brief Constructor for NearestNeighbors class.
         * @details Initializes the NearestNeighbors node and sets up publishers and subscribers.
         */
        NearestNeighbors() : Node("nearest_neighbors") {
            this->declare_parameter<int>("nb_drones");
            this->declare_parameter<double>("neighbor_distance");
            this->declare_parameter<std::vector<double>>("x_init");
            this->declare_parameter<std::vector<double>>("y_init");

            nb_drones = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());
            neighbor_distance = this->get_parameter("neighbor_distance").as_double();
            x_init = this->get_parameter("x_init").as_double_array();
            y_init = this->get_parameter("y_init").as_double_array();
            // Definition of the publishers and the subscribers
            position_subscribers.reserve(nb_drones);
            neighbors_publishers.reserve(nb_drones);
            rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
            auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
            for (auto i{0u}; i < nb_drones; ++i) {
                auto subscriber_topic{"/px4_" + std::to_string(i + 1) + "/fmu/out/vehicle_local_position"};
                position_subscribers.emplace_back(this->create_subscription<VehicleLocalPosition>(subscriber_topic, qos,
                                                                                                  [this, i](
                                                                                                          const VehicleLocalPosition::SharedPtr msg) {
                                                                                                      pose_subscriber_callback(
                                                                                                              msg, i);
                                                                                                  }));
                auto pose_publisher_topic{"/px4_" + std::to_string(i + 1) + "/fmu/out/nearest_neighbors"};
                neighbors_publishers.emplace_back(this->create_publisher<Neighbors>(pose_publisher_topic, 10));

            }

            position_received.reserve(nb_drones);
            position_received.assign(nb_drones, false);
            drones_positions.reserve(nb_drones);
            drones_positions.assign(nb_drones, VehicleLocalPosition{});

            auto timer_callback = [this]() {
                bool all_position_received = std::all_of(position_received.begin(), position_received.end(),
                                                         [](bool value) {
                                                             return value;
                                                         });

                if (all_position_received) {
                    std::transform(position_received.cbegin(), position_received.cend(),
                                   position_received.begin(), // write to the same location
                                   [](const bool is_received) { return false; });

                    find_neighbors();
                }

            };

            timer = this->create_wall_timer(100ms, timer_callback);
        }

    private:
        /**
         * @brief Adjusts the local position to global coordinates based on the initial offsets.
         *
         * This function takes a local position and adjusts it to global coordinates using the initial offsets
         * for the specified drone index.
         *
         * @param localPose The local position to be adjusted.
         * @param drone_idx The index of the drone.
        */
        void local_to_global(VehicleLocalPosition& localPose, const std::size_t drone_idx){
            localPose.x = localPose.x + x_init[drone_idx];
            localPose.y = localPose.y + y_init[drone_idx];
        }

        /**
         * @brief Callback function for the vehicle local position subscriber.
         *
         * This function is called whenever a new vehicle local position message is received.
         * It updates the position for the specified drone index and adjusts it to global coordinates.
         *
         * @param posePtr The shared pointer to the received vehicle local position message.
         * @param drone_idx The index of the drone.
        */
        void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &posePtr, const std::size_t drone_idx) {
            position_received[drone_idx] = true;
            auto pose{*posePtr};
            local_to_global(pose, drone_idx);
            drones_positions[drone_idx] = pose;
        }


        /**
         * @brief Find the nearest neighbors for each drone.
         */
        void find_neighbors() {
            std::vector<Neighbors> neighborhoods;
            neighborhoods.reserve(nb_drones);
            // TODO: Try to optimize the search by using branch and bound or take into account the reciprocity of being neighbors
            std::for_each(std::begin(drones_positions), std::end(drones_positions),
                          [this, &neighborhoods, drone_idx = 0u](const auto &position)mutable {
                              neighborhoods.emplace_back(process_position(drone_idx, position));
                              ++drone_idx;
                          });

            std::for_each(std::begin(neighborhoods), std::end(neighborhoods),
                          [this, drone_idx = 0u](auto &neighborhood) mutable {
                              // This condition is generic because we are checking in the traits that the Neighbors type defines at least a neighbors_position attribute
                              if (!std::empty(neighborhood.neighbors_position)) {
                                  enrich_neighborhood(neighborhood);
                                  this->neighbors_publishers[drone_idx]->publish(neighborhood);
                              }
                              ++drone_idx;
                          });
        }

        /**
         * @brief Process the position of a drone and find its neighbors.
         * @param drone_idx The index of the drone.
         * @param position The position of the drone.
         * @return Neighbors object containing the information of the neighbors.
         */
        Neighbors process_position(const std::size_t &drone_idx, const VehicleLocalPosition &position) {
            Neighbors neighborhood;
            std::for_each(std::begin(this->drones_positions), std::end(this->drones_positions),
                          [this, &neighborhood, position, &drone_idx, neighbor_idx = 0u](
                                  const auto& neighbor_position) mutable {
                              bool is_neighbor{
                                      NearestNeighbors::is_neighbor(position, neighbor_position,
                                                                    this->neighbor_distance)};
                              if (is_neighbor) {
                                  process_neighbor_position(drone_idx, neighbor_idx, position, neighbor_position,
                                                            neighborhood);
                              }
                              ++neighbor_idx;
                              return is_neighbor;
                          });
            process_neighborhood(drone_idx, neighborhood);
            return neighborhood;
        }

    protected:
        /**
         * @brief Process the position of a neighbor and add information to the neighborhood.
         * @param neighbor_idx The index of the neighbor.
         * @param position The position of the drone.
         * @param neighbor_position The position of the neighbor.
         * @param neighborhood The neighborhood to be enriched.
         */
        virtual void process_neighbor_position(const std::size_t drone_idx, const std::size_t neighbor_idx,
                                               const VehicleLocalPosition &position,
                                               VehicleLocalPosition neighbor_position,
                                               Neighbors &neighborhood) {
            // Default implementation proposed (in fact we are sure that the neighbors_attribute exists).
            neighborhood.neighbors_position.emplace_back(neighbor_position);
        };

        /**
         * @brief Process the entire neighborhood and add any additional information if needed.
         * @param drone_idx The index of the drone.
         * @param neighborhood The neighborhood to be processed.
         */
        virtual void process_neighborhood(const std::size_t drone_idx, Neighbors &neighborhood) {};

        /**
         * @brief Enrich the neighborhood with additional information.
         * @param neighborhood The neighborhood to be enriched.
         */
        virtual void enrich_neighborhood(Neighbors &neighborhood) {};

    private:
        /**
         * @brief Check if two positions are within a certain distance, indicating they are neighbors.
         * @param lhs The first position.
         * @param rhs The second position.
         * @param distance The maximum distance for two positions to be considered neighbors.
         * @return True if the positions are neighbors, false otherwise.
         */
        static bool
        is_neighbor(const VehicleLocalPosition &lhs, const VehicleLocalPosition &rhs, const double distance) {
            const auto drone_distance{std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)};
            // The search of neighbors is not really well done yet so you can have the same position in lhs and rhs.
            return ((drone_distance <= distance) && (drone_distance >= 1e-2));
        };

    protected:
        double neighbor_distance{};
        std::size_t nb_drones{};

    private:
        std::vector<NeighborsPublisherSharedPtr> neighbors_publishers;
        std::vector<PositionSubscriberSharedPtr> position_subscribers;
        rclcpp::TimerBase::SharedPtr timer;
        std::vector<bool> position_received;
        std::vector<VehicleLocalPosition> drones_positions;
        std::vector<double> x_init;
        std::vector<double> y_init;
    };
}