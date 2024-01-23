/**
 * @brief File able to arm all the drone in offboard mode assuming they already receive an offboard setpoint
 * @file arming_node.cpp
 * @author Arthur Astier
 */

#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;

// TODO: Try to create a class such as waypoint or the embedded control node can derive from
class Arming : public rclcpp::Node {
    using ArmCheckClient = rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr;
    using VehicleCommand = px4_msgs::msg::VehicleCommand;
public:
    Arming() : Node("arming") {

//        this->declare_parameter<int>("nb_drones");
//        nb_drones = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());
        nb_drones = 4u;
        // Initializing the vectors
        arm_check_clients_.reserve(nb_drones);
        is_armed.reserve(nb_drones);
        is_armed.assign(nb_drones, false);
        is_offboard.reserve(nb_drones);
        is_offboard.assign(nb_drones, false);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_default;
        // Instantiating all the clients and check their availability
        for (auto i{0u}; i < nb_drones; ++i) {
            auto service{"/px4_" + std::to_string(i + 1) + "/fmu/vehicle_command"};
            arm_check_clients_.emplace_back(this->create_client<px4_msgs::srv::VehicleCommand>(
                    service, qos_profile));
            // Wait for the service to become available
            if (!arm_check_clients_[i]->wait_for_service(std::chrono::seconds(5))) {
                RCLCPP_ERROR(this->get_logger(), "Service for drone n°%u not available", i);
            }
            RCLCPP_ERROR(this->get_logger(), "Vehicle command service for drone n°%u available", i);
        }
        auto timer_callback = [this]() {

            bool allArmed = std::all_of(is_armed.begin(), is_armed.end(), [](bool value) {
                return value;
            });
            bool allOffBoard = std::all_of(is_offboard.begin(), is_offboard.end(), [](bool value) {
                return value;
            });
            if (allArmed && allOffBoard) {
                rclcpp::shutdown();
            }
            for (auto i{0u}; i < nb_drones; ++i) {
                if (!is_offboard[i]) {
                    // Set to onboard_mode
                    offboard_mode(i);
                }
                if (!is_armed[i]) {
                    // Set to onboard_mode
                    arm(i);
                }

            }

        };
        timer_ptr_ = this->create_wall_timer(1s, timer_callback);
        // TODO: Check how to set sim_time to true


    }

    void arm(std::size_t drone_idx);

    void offboard_mode(std::size_t drone_idx);

    void disarm();

private:
    std::vector<ArmCheckClient> arm_check_clients_;
    std::vector<bool> is_armed;
    std::vector<bool> is_offboard;
    rclcpp::TimerBase::SharedPtr timer_ptr_;
    std::size_t nb_drones;


    void request_vehicle_command(uint16_t command, std::size_t drone_idx, float param1 = 0.0, float param2 = 0.0);
};

/**
 * @brief Send a command to Arm the vehicle
 */
void Arming::arm(std::size_t drone_idx) {
   request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, drone_idx, 1.0);
    if (is_armed[drone_idx]) {
        RCLCPP_INFO(this->get_logger(), "Drone n°%zu armed !", drone_idx);
    } else {
        RCLCPP_INFO(this->get_logger(), "Didn't succeed to arm the drone n°%zu", drone_idx);
    }
}

/**
 * @brief Send a command to switch in OffBoard Mode
 */
void Arming::offboard_mode(std::size_t drone_idx) {
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, drone_idx, 1, 6);
    if (is_offboard[drone_idx]) {
        RCLCPP_INFO(this->get_logger(), "Drone n°%zu is in offboard mode !", drone_idx);
    } else {
        RCLCPP_INFO(this->get_logger(), "Didn't succeed to switch drone n°%zu to offboard mode", drone_idx);
    }
}

/**
 * @brief Send a command to Disarm the vehicle
 */
void Arming::disarm() {
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);

    RCLCPP_INFO(this->get_logger(), "Disarm command send");
}

/**
 * @brief Request the vehicle command service (Arming, switch to offboard mode...)
 */
void Arming::request_vehicle_command(uint16_t command, std::size_t drone_idx, float param1, float param2) {

    auto request{std::make_shared<px4_msgs::srv::VehicleCommand::Request>()};
    request->request.set__command(command);
    request->request.set__param1(param1);
    request->request.set__param2(param2);
    // We don't know why but target_system make the request wait a lot
//      request->request.target_system = 2;
    request->request.target_component = 1;
    request->request.source_system = 1;
    request->request.source_component = 1;
    request->request.from_external = true;

    // Calculate timestamp
    request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    RCLCPP_INFO(this->get_logger(), "Request sent:");
    // Send the request
    auto result_future{arm_check_clients_[drone_idx]->async_send_request(request,
                                                                         [this, drone_idx](const rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
                                                                             auto& response{future.get()};
                                                                             // Handle the response
                                                                             const uint8_t result{response->reply.result};
                                                                             const uint32_t command{response->reply.command};
                                                                             RCLCPP_INFO(this->get_logger(), "Vehicle Command Service connexion succeeded with result: %d",
                                                                                         result);
                                                                             if (!result) {
                                                                                 if (command==VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM){
                                                                                     is_armed[drone_idx] = true;
                                                                                 }
                                                                                 else if(command==VehicleCommand::VEHICLE_CMD_DO_SET_MODE){
                                                                                     is_offboard[drone_idx] = true;
                                                                                 }
                                                                             }
                                                                         })};

    // Wait for the response without creating deadlock
    result_future.wait_for(100ms);  // timeout to guarantee a graceful finish
}

int main(int argc, char *argv[]) {
    std::cout << "Starting arming drones..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arming>());
    rclcpp::shutdown();
    return 0;
}
