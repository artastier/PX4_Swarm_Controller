/**
* @author Arthur Astier
*/
#include <rclcpp/rclcpp.hpp>
#include "swarm_controller_node.cpp"

class WeightedTopologyController : public SwarmController {
    using OffboardControlMode = SwarmController::OffboardControlMode;
    using Neighbors = SwarmController::Neighbors;
    // TODO: Check to define Neighbors type
public:
    WeightedTopologyController();

private:
    void publish_offboard_control_mode() override;

    void neighbors_callback(const Neighbors::SharedPtr &neighbors) override;
};

WeightedTopologyController::WeightedTopologyController() : SwarmController() {

}

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