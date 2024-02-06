/**
* @author Arthur Astier
*/

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <custom_msgs/msg/neighbors.hpp>

using namespace std::chrono_literals;

class WeightedTopologyController : public rclcpp::Node {
    using Neighbors = custom_msgs::msg::Neighbors;
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;

public:
    WeightedTopologyController();

private:
    void publish_offboard_control_mode();

    void neighbors_callback(const Neighbors::SharedPtr &neighbors);

private:
    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<Neighbors>::SharedPtr neighbors_subscriber;
    rclcpp::TimerBase::SharedPtr timer;

    std::size_t drone_id{};
    std::size_t nb_drones{};

    double x_init{};
    double y_init{};

    double x_formation{};
    double y_formation{};

    // TODO: Check to make a daughter class from this one


};

WeightedTopologyController::WeightedTopologyController() : rclcpp::Node("swarm_controller") {
    const std::string name_space{this->get_namespace()};
    this->declare_parameter<int>("drone_id");
    this->declare_parameter<int>("nb_drones");
    this->declare_parameter<double>("x_init");
    this->declare_parameter<double>("y_init");

    x_init = this->get_parameter("x_init").as_double();
    y_init = this->get_parameter("y_init").as_double();

    drone_id = static_cast<std::size_t>(this->get_parameter("drone_id").as_int());
    nb_drones = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());

    offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
            name_space + "/fmu/in/offboard_control_mode", 10);
    trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
            name_space + "/fmu/in/trajectory_setpoint", 10);

    neighbors_subscriber = this->create_subscription<Neighbors>(name_space + "/fmu/out/nearest_neighbors", 10,
                                                                [this](const Neighbors::SharedPtr neighbors) {
                                                                    neighbors_callback(neighbors);
                                                                });
    auto timer_callback = [this]() {
        publish_offboard_control_mode();
        // Check to send a default position setpoint and when receiving neighbors using the control law
    };
    timer = this->create_wall_timer(100ms, timer_callback);
}
// TODO: Check to make these methods (publish_offboard_control_mode and neighbors_callback) virtuals.
/**
 * @brief Publish the offboard control mode.
 */
void WeightedTopologyController::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = true;
    // TODO: Check if velocity control is needed
    msg.velocity = false;
    msg.acceleration = false;
    msg.attitude = false;
    msg.body_rate = false;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    offboard_control_mode_publisher_->publish(msg);
}

/**
 *
 * @brief
 */
void WeightedTopologyController::neighbors_callback(const Neighbors::SharedPtr &neighbors) {

}

/**
 *
 * @param argc
 * @param argv
 * @return
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightedTopologyController>());
    rclcpp::shutdown();
    return 0;
}