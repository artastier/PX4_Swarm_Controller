#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <yaml-cpp/yaml.h>

// TODO: Try to refactor code using template and traits on the Setpoint and the Pose
class ChangeWaypoint : public rclcpp::Node {
    using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;
    using VehicleLocalPosition = px4_msgs::msg::VehicleLocalPosition;
public:
    ChangeWaypoint() : Node("waypoint") {
        // TODO: Set use_sim_time true
        const std::string name_space{this->get_namespace()};
        this->declare_parameter<std::string>("wp_path");
        this->declare_parameter<double>("x_init");
        this->declare_parameter<double>("y_init");

        const auto wp_path{this->get_parameter("wp_path").as_string()};
        x_init = this->get_parameter("x_init").as_double();
        y_init = this->get_parameter("y_init").as_double();

//        const std::string name_space{"/px4_1"};
        offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
                name_space + "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
                name_space + "/fmu/in/trajectory_setpoint",
                10);
        // See PX4 documentation on subscriber
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 10), qos_profile);
        pose_subscriber = this->create_subscription<VehicleLocalPosition>(
                name_space + "/fmu/out/vehicle_local_position",
                qos, [this](const typename VehicleLocalPosition::SharedPtr msg) {
                    pose_subscriber_callback(msg);
                });
        const auto node{YAML::LoadFile(wp_path)};
        threshold = node["threshold"].as<double>();
        RCLCPP_INFO(this->get_logger(), "WP Threshold: %f", threshold);
        threshold_angle = node["threshold_angle"].as<double>();
        RCLCPP_INFO(this->get_logger(), "WP Angle Threshold: %f", threshold_angle);
        waypoints = node["wp"][name_space];
        RCLCPP_INFO(this->get_logger(), "Found %ld waypoints", std::size(waypoints));
        // Initialize the first waypoint
        writeWP(wp_idx);
    }

private:
    void publish_offboard_control_mode() {
        OffboardControlMode msg{};
        msg.position = true;
        msg.velocity = false;
        msg.acceleration = false;
        msg.attitude = false;
        msg.body_rate = false;
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        offboard_control_mode_publisher_->publish(msg);
    }

    void pose_subscriber_callback(const VehicleLocalPosition::SharedPtr& pose) {
        double theta_d = coord(wp_idx, "yaw");

        double distance{std::hypot(pose->x - waypoint.position[0], pose->y - waypoint.position[1], pose->z - waypoint.position[2])};

        double current_theta{pose->heading};

        if ((distance < threshold) && (abs(fmod(current_theta - theta_d + M_PI, 2 * M_PI) - M_PI) < threshold_angle)) {
            wp_idx++;
            if (wp_idx == std::size(waypoints)) {
                wp_idx = 0;
            }
            writeWP(wp_idx);
        }
        publish_offboard_control_mode();
        trajectory_setpoint_publisher_->publish(waypoint);
    }

    double coord(const size_t idx, const std::string &var) {
        return waypoints[idx][var].as<double>();
    }

    void writeWP(const std::size_t idx) {
        // TODO: Check use_sim_time
        waypoint.timestamp = static_cast<uint64_t>(this->get_clock()->now().seconds());
        // North-East-Down frame (different from Gazebo)
        waypoint.position = {static_cast<float>(coord(idx, "y") - y_init), static_cast<float>(coord(idx, "x") - x_init),
                             static_cast<float>(coord(idx, "z"))};

        waypoint.yaw = static_cast<float>(coord(idx, "yaw"));

    }

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Subscription<VehicleLocalPosition>::SharedPtr pose_subscriber;

    TrajectorySetpoint waypoint;
    std::size_t wp_idx{0u};
    YAML::Node waypoints;
    double threshold{};
    double threshold_angle{};

    double x_init{};
    double y_init{};


};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ChangeWaypoint>());
    rclcpp::shutdown();
    return 0;
}