/**
* @author Arthur Astier
*/
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <chrono>
#include <custom_msgs/msg/neighbors.hpp>

using namespace std::chrono_literals;

class NearestNeighbors : public rclcpp::Node {
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
    using PositionSubscriberSharedPtr = rclcpp::Subscription<VehicleLocalPosition>::SharedPtr;
    using Neighbors = custom_msgs::msg::Neighbors;
    using NeighborsPublisherSharedPtr = rclcpp::Publisher<Neighbors>::SharedPtr;
public:
    /**
     * @brief
     */
    NearestNeighbors();

private:
    /**
    * @brief
    */
    void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose, const std::size_t drone_idx);

    /**
     * @brief
     */
    void find_neighbors();

    /**
     * @brief
     */
    static bool is_neighbor(const VehicleLocalPosition &lhs, const VehicleLocalPosition &rhs, const double distance);

private:
    std::vector<PositionSubscriberSharedPtr> position_subscribers;
    std::vector<NeighborsPublisherSharedPtr> neighbors_publishers;
    rclcpp::TimerBase::SharedPtr timer;
    // Parameters
    std::vector<bool> leaders;
    std::vector<bool> position_received;
    std::vector<VehicleLocalPosition> drones_positions;
    double neighbor_distance{};
    std::size_t nb_drones{};


};

NearestNeighbors::NearestNeighbors() : Node("nearest_neighbors") {
    // Definition of the parameters required for a leader-follower control (i.e you can remove the "is_leader"
    // parameter if neeeded)
    this->declare_parameter<int>("nb_drones");
    this->declare_parameter<double>("neighbor_distance");
    this->declare_parameter<std::vector<bool>>("leaders");

    nb_drones = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());
    neighbor_distance = this->get_parameter("neighbor_distance").as_double();
    // TODO: Check if it works
    leaders = this->get_parameter("leaders").as_bool_array();
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
        auto publisher_topic{"/px4_" + std::to_string(i + 1) + "/fmu/out/nearest_neighbors"};
        // TODO: Understand why using these publishers doesn't work
        neighbors_publishers.emplace_back(this->create_publisher<Neighbors>(publisher_topic, 10));
    }

    position_received.reserve(nb_drones);
    position_received.assign(nb_drones, false);
    drones_positions.reserve(nb_drones);
    drones_positions.assign(nb_drones, VehicleLocalPosition{});

    auto timer_callback = [this]() {
        bool all_position_received = std::all_of(position_received.begin(), position_received.end(), [](bool value) {
            return value;
        });

        // Shutdown if all drones are armed and in offboard mode
        if (all_position_received) {
            std::transform(position_received.cbegin(), position_received.cend(),
                           position_received.begin(), // write to the same location
                           [](const bool is_received) { return false; });

            find_neighbors();
        }

    };

    // Create a timer to periodically check and arm the drones
    timer = this->create_wall_timer(100ms, timer_callback);

};

/**
 * @brief Callback function for the vehicle local position subscriber.
 * @param pose The received vehicle local position message.
 */
void
NearestNeighbors::pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose, const std::size_t drone_idx) {
    position_received[drone_idx] = true;
    drones_positions[drone_idx] = *pose;
}

/**
 * @brief
 */
void NearestNeighbors::find_neighbors() {
    // TODO: Try to optimize the search by using branch and bound or take into account the reciprocity of being neighbors
    std::for_each(std::begin(drones_positions), std::end(drones_positions),
                  [this, drone_idx = 0u](const auto &position)mutable {
                      std::vector<VehicleLocalPosition> neighbors_positions;
                      std::vector<bool> neighbors_leaders;
                      std::copy_if(std::begin(this->drones_positions), std::end(this->drones_positions),
                                   std::back_inserter(neighbors_positions),
                                   [this, &neighbors_leaders, position, neighbor_idx = 0u](
                                           const auto &neighbor_position) mutable {
                                       bool is_neighbor{
                                               NearestNeighbors::is_neighbor(position, neighbor_position,
                                                                             this->neighbor_distance)};
                                       if (is_neighbor) {
                                           const bool is_a_leader{this->leaders[neighbor_idx]};
                                           neighbors_leaders.emplace_back(is_a_leader);
                                       }
                                       ++neighbor_idx;
                                       return is_neighbor;
                                   });
                      Neighbors nearest_neighbors;
                      if ((!std::empty(neighbors_leaders)) && (!std::empty(neighbors_positions))) {
                          nearest_neighbors.set__neighbors_leaders(neighbors_leaders);
                          nearest_neighbors.set__neighbors_position(neighbors_positions);
                          this->neighbors_publishers[drone_idx]->publish(nearest_neighbors);
                      }
                      ++drone_idx;

                  });
}

bool
NearestNeighbors::is_neighbor(const VehicleLocalPosition &lhs, const VehicleLocalPosition &rhs, const double distance) {
    const auto drone_distance{std::hypot(lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z)};
    // The search of neighbors is not really well done yet so you can have the same position in lhs and rhs.
    return ((drone_distance <= distance) && (drone_distance != 0));
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NearestNeighbors>());
    rclcpp::shutdown();
    return 0;
}