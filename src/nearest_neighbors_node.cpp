/**
* @author Arthur Astier
*/
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <custom_msgs/msg/neighbors.hpp>

class NearestNeighbors : public rclcpp::Node {
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
    using PositionSubscriberSharedPtr = rclcpp::Subscription<VehicleLocalPosition>::SharedPtr;
    using Neighbors = custom_msgs::msg::Neighbors;
    using NeighborsPubliserSharedPtr = rclcpp::Publisher<Neighbors>::SharedPtr;
public:
    /**
     * @brief
     */
    NearestNeighbors();

private:
    /**
    * @brief Callback function for the vehicle local position subscriber.
    * @param pose The received vehicle local position message.
    */
    void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose, const std::size_t drone_idx);

private:
    std::vector<PositionSubscriberSharedPtr> position_subscribers;
    std::vector<NeighborsPubliserSharedPtr> neighbors_publishers;
    // Parameters
    std::vector<bool> is_leader;
    std::vector<bool> all_position_received;
    std::vector<bool> drones_positions;
    double neighbor_distance{};
    std::size_t nb_drones{};


};

NearestNeighbors::NearestNeighbors() : Node("nearest_neighbors") {

    // Definition of the parameters required for a leader-follower control (i.e you can remove the "is_leader"
    // parameter if neeeded)
    this->declare_parameter<int>("nb_drones");
    this->declare_parameter<double>("neighbor_distance");
    this->declare_parameter<std::vector<bool>>("is_leader");

    nb_drones = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());
    neighbor_distance = this->get_parameter("neighbor_distance").as_double();
    // TODO: Check if it works
    is_leader = this->get_parameter("is_leader").as_bool_array();

    // Definition of the publishers and the subscribers
    position_subscribers.reserve(nb_drones);
    neighbors_publishers.reserve(nb_drones);
    for (auto i{0u}; i < nb_drones; ++i) {
        auto subscriber_topic{"/px4_" + std::to_string(i + 1) + "/fmu/out/vehicle_local_position"};
        position_subscribers.emplace_back(this->create_subscription<VehicleLocalPosition>(subscriber_topic, 10,
                                                                                          [this, i](
                                                                                                  const VehicleLocalPosition::SharedPtr msg) {
                                                                                              pose_subscriber_callback(
                                                                                                      msg, i);
                                                                                          }));
        auto publisher_topic{"/px4_" + std::to_string(i + 1) + "/fmu/out/nearest_neighbors"};
        neighbors_publishers.emplace_back(this->create_publisher<Neighbors>(publisher_topic, 10));
    }

    all_position_received.reserve(nb_drones);
    drones_positions.reserve(nb_drones);

};

/**
 * @brief Callback function for the vehicle local position subscriber.
 * @param pose The received vehicle local position message.
 */
void
NearestNeighbors::pose_subscriber_callback(const VehicleLocalPosition::SharedPtr &pose, const std::size_t drone_idx) {

    double distance{std::hypot(pose->x, pose->y,
                               pose->z)};

}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NearestNeighbors>());
    rclcpp::shutdown();
    return 0;
}