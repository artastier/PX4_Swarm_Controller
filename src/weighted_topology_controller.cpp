/**
 * @brief  This controller is based on the paper:
 *
 * Distributed leader-follower formation control for multiple quadrotors with weighted topology,
 *
 * Zhicheng Hou, Isabelle Fantoni, hal-01180491
 *
 * @author Arthur Astier
*/
#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <eigen3/Eigen/Eigen>
#include "swarm_controller_node.cpp"

class WeightedTopologyController : public SwarmController {
    using OffboardControlMode = SwarmController::OffboardControlMode;
    using Neighbors = SwarmController::Neighbors;
    using PRCS = Eigen::Vector<std::size_t, Eigen::Dynamic>;
    using Weights = Eigen::VectorXd;
public:
    WeightedTopologyController();

private:
    void publish_offboard_control_mode() override;

    void neighbors_callback(const Neighbors::SharedPtr &neighbors) override;

    void timer_callback() override;

private:
    PRCS prcs;
    Weights weights;

    std::size_t drone_id{};
    std::size_t nb_drones{};

    double x_init{};
    double y_init{};

};

WeightedTopologyController::WeightedTopologyController() : SwarmController() {
    // TODO: Add the distances of all the drones to the leader as a parameter
    this->declare_parameter<int>("drone_id");
    this->declare_parameter<int>("nb_drones");
    this->declare_parameter<double>("x_init");
    this->declare_parameter<double>("y_init");


    x_init = this->get_parameter("x_init").as_double();
    y_init = this->get_parameter("y_init").as_double();

    drone_id = static_cast<std::size_t>(this->get_parameter("drone_id").as_int());
    nb_drones = static_cast<std::size_t>(this->get_parameter("nb_drones").as_int());

    prcs = PRCS::Zero(nb_drones);
    weights = Weights::Zero(nb_drones);
}

/**
 * @brief Publish the offboard control mode.
 */
void WeightedTopologyController::publish_offboard_control_mode() {
    OffboardControlMode msg{};
    msg.position = false;
    msg.velocity = false;
    // In the paper, the command is a command of acceleration
    msg.acceleration = true;
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
 * @brief
 */
void WeightedTopologyController::timer_callback() {
    // TODO: Handle when no neighbors are detected (at the beginning for example)
    publish_offboard_control_mode();
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