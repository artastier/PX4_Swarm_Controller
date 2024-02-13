/**
 * @brief Template class for implementing a swarm controller for drone fleets.
 *
 * This class provides a template for creating swarm controllers to manage the behavior
 * of a fleet of drones operating in a coordinated manner. It serves as a base class
 * for implementing specific swarm control algorithms and behaviors.
 *
 * Subclasses of SwarmController must implement the abstract methods `neighbors_callback`
 * and `timer_callback` to handle incoming neighbor data and perform periodic control
 * computations, respectively.
 *
 * @tparam Neighbors Type representing the neighbors of a drone, providing information
 *                   about their positions or states.
 * @author Arthur Astier
 */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include "NeighborsTraits.hpp"
#include <chrono>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>

using namespace std::chrono_literals;

/**
 * @brief Enum representing control modes.
 */
enum class CONTROL{
    // For other type of control you need to send a different message than a trajectory setpoint
    // See https://docs.px4.io/main/en/flight_modes/offboard.html
    POSITION = 0, /**< Position control */
    VELOCITY = 1, /**< Velocity control */
    ACCELERATION = 2, /**< Acceleration control */
};

namespace Controller {
    /**
     * @brief Template class for a swarm controller.
     * @tparam Neighbors Type representing the neighbors of a drone.
     */
    template<typename Neighbors>
    class SwarmController : public rclcpp::Node {
        // This trait ensures checking std::empty(neighborhood.neighbors_position) won't crash
        static_assert(traits::has_neighbors_position_attribute_and_is_VLP_v<Neighbors>,
                      "Neighbors type must have neighbors_position attribute of type vector<px4_msgs::msg::VehicleLocalPosition>");
        static_assert(traits::has_shared_ptr_v<Neighbors>, "The Neighbors type doesn't define ::SharedPtr");
        using OffboardControlMode = px4_msgs::msg::OffboardControlMode;
    protected:
        using TrajectorySetpoint = px4_msgs::msg::TrajectorySetpoint;

    public:
        /**
         * @brief Constructor.
         */
        SwarmController() : rclcpp::Node("swarm_controller") {
            const std::string name_space{this->get_namespace()};
            offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>(
                    name_space + "/fmu/in/offboard_control_mode", 10);
            trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>(
                    name_space + "/fmu/in/trajectory_setpoint", 10);

            neighbors_subscriber = this->create_subscription<Neighbors>(name_space + "/fmu/out/nearest_neighbors", 10,
                                                                        [this](const typename Neighbors::SharedPtr neighbors) {
                                                                            neighbors_callback(neighbors);
                                                                        });

            timer = this->create_wall_timer(100ms, [this]() { timer_callback(); });
        }

    protected:
        /**
         * @brief Publishes the offboard control mode.
         * @param control The control mode to publish.
         */
        void publish_offboard_control_mode(const CONTROL &control) {
            OffboardControlMode msg{};
            switch (control) {
                case CONTROL::POSITION:
                    msg.position = true;
                    break;
                case CONTROL::VELOCITY:
                    msg.velocity = true;
                    break;
                case CONTROL::ACCELERATION:
                    msg.acceleration = true;
                    break;
            }
            msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
            offboard_control_mode_publisher_->publish(msg);
        }

    private:
        /**
         * @brief Callback function for receiving neighbors data.
         * @param neighbors The received neighbors data.
         */
        virtual void neighbors_callback(const typename Neighbors::SharedPtr &neighbors) = 0;

        /**
         * @brief Timer callback function.
         */
        virtual void timer_callback() = 0;

    protected:
        rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    private:
        rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
        typename rclcpp::Subscription<Neighbors>::SharedPtr neighbors_subscriber;
        rclcpp::TimerBase::SharedPtr timer;

    };
}