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

class SwarmController : public rclcpp::Node {
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
public:
    using Neighbors = custom_msgs::msg::Neighbors;
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;

public:
    SwarmController();

private:
    virtual void publish_offboard_control_mode() = 0;

    virtual void neighbors_callback(const Neighbors::SharedPtr &neighbors) = 0;

protected:
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

SwarmController::SwarmController() : rclcpp::Node("swarm_controller") {
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

