/**
* @author Arthur Astier
*/
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <custom_msgs/msg/neighbors.hpp>

using namespace std::chrono_literals;

// TODO: Check if it would be more efficient to directly template over the Neighbors type
template<typename Neighbors>
class NearestNeighbors : public rclcpp::Node {
protected:
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
private:
    using PositionSubscriberSharedPtr = rclcpp::Subscription<VehicleLocalPosition>::SharedPtr;
    using NeighborsPublisherSharedPtr = typename rclcpp::Publisher<Neighbors>::SharedPtr;
public:
    /**
     * @brief
     */
    NearestNeighbors() : Node("nearest_neighbors") {
        // Definition of the parameters required for a leader-follower control (i.e you can remove the "is_leader"
        // parameter if neeeded)
        this->declare_parameter<int>("nb_drones");
        this->declare_parameter<double>("neighbor_distance");

        const auto nb_drones {static_cast<std::size_t>(this->get_parameter("nb_drones").as_int())};
        neighbor_distance = this->get_parameter("neighbor_distance").as_double();

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
    * @brief Callback function for the vehicle local position subscriber.
    * @param pose The received vehicle local position message.
    */
    void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose, const std::size_t drone_idx) {
        position_received[drone_idx] = true;
        drones_positions[drone_idx] = *pose;
    }

    /**
     * @brief
     */
    void find_neighbors() {
        // TODO: Try to optimize the search by using branch and bound or take into account the reciprocity of being neighbors
        std::for_each(std::begin(drones_positions), std::end(drones_positions),
                      [this, drone_idx = 0u](const auto &position)mutable {
                          process_position(drone_idx, position);
                          ++drone_idx;
                      });
    }

    /**
     * @brief
     */
    void process_position(const std::size_t drone_idx, const VehicleLocalPosition &position) {
        Neighbors nearest_neighbors;
        std::copy_if(std::begin(this->drones_positions), std::end(this->drones_positions),
                     std::back_inserter(nearest_neighbors.neighbors_position),
                     [this, &nearest_neighbors, position, neighbor_idx = 0u](
                             const auto &neighbor_position) mutable {
                         bool is_neighbor{
                                 process_neighbor_position(neighbor_idx, position, neighbor_position,
                                                           nearest_neighbors)};
                         ++neighbor_idx;
                         return is_neighbor;
                     });
        // TODO: Create virtual function to check if the msg is empty
        if (!std::empty(nearest_neighbors.neighbors_ids)) {
            this->neighbors_publishers[drone_idx]->publish(nearest_neighbors);
        }
    }

    /**
     * @breif
     */
    virtual bool process_neighbor_position(const std::size_t neighbor_idx, const VehicleLocalPosition &position,
                                           const VehicleLocalPosition &neighbor_position,
                                           Neighbors &nearest_neighbors) = 0;
protected:
    /**
     * @brief
     */
    static bool is_neighbor(const VehicleLocalPosition &lhs, const VehicleLocalPosition &rhs, const double distance) {
        const auto drone_distance{std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)};
        // The search of neighbors is not really well done yet so you can have the same position in lhs and rhs.
        return ((drone_distance <= distance) && (drone_distance != 0));
    };

protected:
    double neighbor_distance{};
private:
    std::vector<NeighborsPublisherSharedPtr> neighbors_publishers;
    std::vector<PositionSubscriberSharedPtr> position_subscribers;
    rclcpp::TimerBase::SharedPtr timer;
    // Parameters
    std::vector<bool> position_received;
    std::vector<VehicleLocalPosition> drones_positions;
};

