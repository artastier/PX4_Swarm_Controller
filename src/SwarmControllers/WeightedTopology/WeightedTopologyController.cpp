/**
 * @brief Controller implementing distributed leader-follower formation control for multiple quadrotors with weighted topology.
 *
 * This controller is based on the paper:
 *
 * "Distributed leader-follower formation control for multiple quadrotors with weighted topology"
 * Zhicheng Hou, Isabelle Fantoni, hal-01180491
 *
 * It extends the SwarmController class to implement specific behavior for the weighted topology scenario.
 *
 * @author Arthur Astier
 */

#include "SwarmControllers/WeightedTopology/WeightedTopologyController.hpp"

/**
 * @brief Constructs a new Weighted Topology Controller object.
 *
 * Initializes the controller by retrieving gains from parameters.
 */
Controller::WeightedTopologyController::WeightedTopologyController() : SwarmController() {
    this->declare_parameter<std::vector<double>>("gains");

    const auto gains{this->get_parameter("gains").as_double_array()};
    if (std::size(gains)!=9){
        RCLCPP_INFO(this->get_logger(),"You didn't provide Kp, Ki, Kd for each PID (9 gains to provide)");
    }else{
        pid_ax.setKp(gains[0]);
        pid_ax.setKi(gains[1]);
        pid_ax.setKd(gains[2]);
        pid_ay.setKp(gains[3]);
        pid_ay.setKi(gains[4]);
        pid_ay.setKd(gains[5]);
        pid_az.setKp(gains[6]);
        pid_az.setKi(gains[7]);
        pid_az.setKd(gains[8]);
    }

}

/**
 * @brief Convert neighbors data to matrix representation.
 *
 * @param neighbors The neighbors data received from the subscription.
 */
void Controller::WeightedTopologyController::neighbors_to_matrix(
        const WeightedTopologyController::WeightedTopologyNeighbors &neighbors) {
    const auto neighbors_position{neighbors.neighbors_position};
    const auto weights{neighbors.weights};
    neighborhood = Neighborhood::Zero(6, std::size(neighbors_position));
    std::for_each(std::begin(neighbors_position), std::end(neighbors_position),
                  [&, idx = 0u](const auto &position) mutable {
                      const PoseTwist neighborPoseTwist{position.x, position.vx, position.y, position.vy, position.z,
                                                        position.vz};
                      neighborhood.block(0, idx, 6, 1) = weights[idx] * neighborPoseTwist;
                      ++idx;
                  });
}

/**
 * @brief Callback function for processing incoming neighbor data.
 *
 * @param neighbors Pointer to the received neighbor data.
 */
void Controller::WeightedTopologyController::neighbors_callback(const WeightedTopologyNeighbors::SharedPtr &neighbors) {
    if (!std::empty(neighbors->neighbors_position)) {
        is_neighborhood_empty = false;
        const auto vect_neighborhood{*neighbors};
        neighbors_to_matrix(vect_neighborhood);
    } else {
        is_neighborhood_empty = true;
    }
}

/**
* @brief Timer callback function for periodic control computations.
*/
void Controller::WeightedTopologyController::timer_callback() {
    TrajectorySetpoint setpoint{};
    if (!is_neighborhood_empty) {
        // In fact, when there are no neighbors, there are no more messages published on the topic and
        // is_neighborhood_empty remains locked at false.
        is_neighborhood_empty = true;
        publish_offboard_control_mode(CONTROL::ACCELERATION);
        compute_command(setpoint);
    } else {
        command_tp = 0.0;
        publish_offboard_control_mode(CONTROL::POSITION);
        setpoint.position = {default_pose[0], default_pose[1], default_pose[2]};
    }
    setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(setpoint);
}


/**
 * @brief Compute control command based on the current neighborhood information.
 *
 * @param setpoint Reference to the trajectory setpoint to be updated with the computed command.
 */
void Controller::WeightedTopologyController::compute_command(TrajectorySetpoint &setpoint) {
    // TODO: Check North East Down in the neighbors when subtracting dij and here when sending in the command + DO UTest in debug with launching it int he launch file
    const PoseTwist RPVVs{neighborhood.rowwise().sum()};
    // [x, x_dot, y, y_dot, z, z_dot] => reshape to (2,3) [[-x,-x_dot],[-y,-y_dot],[-x,-z_dot]]
    // Then sum the rows
    const Eigen::RowVector3f command{-RPVVs.reshaped(2, 3).colwise().sum()};
    // In offboard mode, if you want to send a command in acceleration you need to send nan on the position and the velocity
    // See https://docs.px4.io/main/en/flight_modes/offboard.html
    setpoint.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::quiet_NaN()};
    setpoint.velocity = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::quiet_NaN()};
    const auto now_tp{this->get_clock()->now().seconds()/1000};
    auto dt{0.};
    if (command_tp!=0){
        dt = now_tp-command_tp;
    }else{
        pid_ax.reset();
        pid_ay.reset();
        pid_az.reset();
        dt = command_tp;
    }
    const auto x_updated_command{pid_ax.update(command[0],dt)};
    const auto y_updated_command{pid_ay.update(command[1],dt)};
    const auto z_updated_command{pid_az.update(command[2],dt)};
    setpoint.acceleration = {x_updated_command, y_updated_command, z_updated_command};
    command_tp = now_tp;
}


/**
 * @brief Main function to run the controller.
 *
 * Initializes the ROS 2 node and starts spinning the node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Command-line arguments.
 * @return int Exit status.
 */
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Controller::WeightedTopologyController>());
    rclcpp::shutdown();
    return 0;
}