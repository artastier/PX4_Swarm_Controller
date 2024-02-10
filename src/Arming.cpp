/**
 * @author Arthur Astier
 */

#include "Arming.hpp"

Arming::Arming() : Node("arming") {
    // Retrieve the number of drones from the parameter server
    this->declare_parameter<int>("nb_drones");
    nb_drones = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());

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
            RCLCPP_ERROR(this->get_logger(), "Service for drone n°%u not available", i + 1);
        }

        RCLCPP_INFO(this->get_logger(), "Vehicle command service for drone n°%u available", i + 1);
    }

    auto timer_callback = [this]() {
        // Check if all drones are armed and in offboard mode
        bool allArmed = std::all_of(is_armed.begin(), is_armed.end(), [](bool value) {
            return value;
        });

        bool allOffBoard = std::all_of(is_offboard.begin(), is_offboard.end(), [](bool value) {
            return value;
        });

        // Shutdown if all drones are armed and in offboard mode
        if (allArmed && allOffBoard) {
            rclcpp::shutdown();
        }

        // Arm and set to offboard mode for each drone
        for (auto i{0u}; i < nb_drones; ++i) {
            if (!is_offboard[i]) {
                // Set to offboard mode
                offboard_mode(i);
            }

            if (!is_armed[i]) {
                // Arm the drone
                arm(i);
            }
        }
    };

    // Create a timer to periodically check and arm the drones
    timer_ptr_ = this->create_wall_timer(1s, timer_callback);
}


/**
 * @brief Arm the specified drone.
 * @param drone_idx Index of the drone to be armed.
 */
void Arming::arm(std::size_t drone_idx) {
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, drone_idx, 1.0);
}

/**
 * @brief Set the specified drone to offboard mode.
 * @param drone_idx Index of the drone to be set to offboard mode.
 */
void Arming::offboard_mode(std::size_t drone_idx) {
    request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, drone_idx, 1, 6);
}

/**
 * @brief Request a vehicle command for the specified drone.
 * @param command Vehicle command to be requested.
 * @param drone_idx Index of the drone.
 * @param param1 Command parameter 1.
 * @param param2 Command parameter 2.
 */
void Arming::request_vehicle_command(uint16_t command, std::size_t drone_idx, float param1, float param2) {
    auto request{std::make_shared<px4_msgs::srv::VehicleCommand::Request>()};
    request->request.set__command(command);
    request->request.set__param1(param1);
    request->request.set__param2(param2);
    request->request.target_component = 1;
    request->request.source_system = 1;
    request->request.source_component = 1;
    request->request.from_external = true;

    // Calculate timestamp
    request->request.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    // Send the request
    auto result_future{arm_check_clients_[drone_idx]->async_send_request(
            request, [this, drone_idx](const rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) {
                auto& response{future.get()};
                const uint8_t result{response->reply.result};
                const uint32_t command{response->reply.command};

                if (!result) {
                    if (command == VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM) {
                        is_armed[drone_idx] = true;
                        RCLCPP_INFO(this->get_logger(), "Drone n°%zu is armed!", drone_idx + 1);
                    } else if (command == VehicleCommand::VEHICLE_CMD_DO_SET_MODE) {
                        is_offboard[drone_idx] = true;
                        RCLCPP_INFO(this->get_logger(), "Drone n°%zu is in offboard mode!", drone_idx + 1);
                    }
                }
            })};

    // Wait for the response without creating a deadlock
    result_future.wait_for(100ms);  // Timeout to guarantee a graceful
}

/**
 * @brief Main function to start the arming node.
 * @param argc Number of command line arguments.
 * @param argv Array of command line arguments.
 * @return Exit code.
 */
int main(int argc, char *argv[]) {
    std::cout << "Starting arming drones..." << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Arming>());
    rclcpp::shutdown();
    return 0;
}
