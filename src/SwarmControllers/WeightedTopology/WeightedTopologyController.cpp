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
    // TODO: Try to use dynamic reconfigure
//    this->declare_parameter<std::vector<double>>("gains");
//
//    const auto vect_gains{this->get_parameter("gains").as_double_array()};
    const std::vector<double> vect_gains{0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    if (std::size(vect_gains) == 6) {
        gains = Gains{vect_gains[0], vect_gains[1], vect_gains[2], vect_gains[3], vect_gains[4], vect_gains[5]};
    } else {
        RCLCPP_INFO(this->get_logger(), "You didn't provide the right amount of gain!");
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
    // - K * (term-to-term) [x, x_dot, y, y_dot, z, z_dot] => reshape to (2,3) [[-K1*x,-K2*x_dot],[-K3*y,-K4*y_dot],[-K5*x,-K6*z_dot]]
    // Then sum the rows
    const Eigen::RowVector3f command{((-gains.array() * RPVVs.array()).matrix()).reshaped(2, 3).colwise().sum()};
    // In offboard mode, if you want to send a command in acceleration you need to send nan on the position and the velocity
    // See https://docs.px4.io/main/en/flight_modes/offboard.html
    setpoint.position = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::quiet_NaN()};
    setpoint.velocity = {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(),
                         std::numeric_limits<float>::quiet_NaN()};
    setpoint.acceleration = {command[0], command[1], command[2]};
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