/**
* @author Arthur Astier
*/

#include "SwarmControllers/WeightedTopology/WeightedTopologyController.hpp"

// TODO: Add doxygen
WeightedTopologyController::WeightedTopologyController() : SwarmController() {
    // TODO: Try to use dynamic reconfigure
    this->declare_parameter<std::vector<double>>("gains");

    const auto vect_gains{this->get_parameter("gains").as_double_array()};
    if (std::size(vect_gains)==6){
        gains = Gains{vect_gains[0],vect_gains[1],vect_gains[2],vect_gains[3],vect_gains[4],vect_gains[5]};
    }else{
        RCLCPP_INFO(this->get_logger(),"You didn't provide the right amount of gain!");
    }
}

/**
 * @brief
 * @param neighbors
 */
void WeightedTopologyController::neighbors_to_matrix(
        const WeightedTopologyController::WeightedTopologyNeighbors &neighbors) {
    const auto neighbors_position{neighbors.neighbors_position};
    const auto weights{neighbors.weights};
    neighborhood = Neighborhood::Zero(6, std::size(neighbors_position));
    std::for_each(std::begin(neighbors_position), std::end(neighbors_position),
                  [&, idx = 0u](const auto &position) mutable {
                      const PoseTwist neighborPoseTwist{position.x, position.vx, position.y, position.vy, position.z,
                                                        position.vz};
                      neighborhood.block(0,idx,6,1) = weights[idx] * neighborPoseTwist;
                      ++idx;
                  });
}

/**
 *
 * @brief
 */
void WeightedTopologyController::neighbors_callback(const WeightedTopologyNeighbors::SharedPtr &neighbors) {
    if (!std::empty(neighbors->neighbors_position)) {
        is_neighborhood_empty = false;
        const auto vect_neighborhood{*neighbors};
        neighbors_to_matrix(vect_neighborhood);
    } else {
        is_neighborhood_empty = true;
    }
}

/**
 * @brief
 */
void WeightedTopologyController::timer_callback() {
    TrajectorySetpoint setpoint{};
    if (!is_neighborhood_empty) {
        publish_offboard_control_mode(CONTROL::ACCELERATION);
        compute_command(setpoint);
    } else {
        publish_offboard_control_mode(CONTROL::POSITION);
        setpoint.position = {default_pose[0], default_pose[1], default_pose[2]};
    }
    setpoint.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(setpoint);
}

void WeightedTopologyController::compute_command(TrajectorySetpoint& setpoint) {
    const PoseTwist RPVVs{neighborhood.rowwise().sum()};
    // - K * (term-to-term) [x, x_dot, y, y_dot, z, z_dot] => reshape to (2,3) [[-K1*x,-K2*x_dot],[-K3*y,-K4*y_dot],[-K5*x,-K6*z_dot]]
    // Then sum the rows
    const Eigen::RowVector3f command{((-gains.array() * RPVVs.array()).matrix()).reshaped(2, 3).colwise().sum()};
    setpoint.acceleration = {command[0], command[1], command[2]};
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