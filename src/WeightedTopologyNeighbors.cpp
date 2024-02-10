/**
 * @author Arthur Astier
 */

#include "WeightedTopologyNeighbors.hpp"

// TODO: Generate doxygen
WeightedTopologyNeighbors::WeightedTopologyNeighbors()
        : NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors>() {
    this->declare_parameter<std::vector<bool>>("leaders");
    leaders = this->get_parameter("leaders").as_bool_array();
    // Initialization provided in the Figure 2 of the paper. We assume they are not neighbors at the beginning.
    prcs = PRCS::Constant(nb_drones, nb_drones);
    prcs_neighborhood = PRCS::Constant(nb_drones, nb_drones);
}

void WeightedTopologyNeighbors::process_neighbor_position(const std::size_t neighbor_idx,
                                                          const VehicleLocalPosition &position,
                                                          const VehicleLocalPosition &neighbor_position,
                                                          WeightedTopologyNeighborsMsg &neighborhood) {

    neighborhood.neighbors_ids.emplace_back(neighbor_idx);
    prcs_neighborhood[neighbor_idx] = prcs[neighbor_idx];
}

void WeightedTopologyNeighbors::process_neighborhood(const std::size_t drone_idx,
                                                     WeightedTopologyNeighborsMsg &neighborhood) {
    if (!std::empty(neighborhood.neighbors_position)) {
        if (leaders[drone_idx]) {
            prcs[drone_idx] = 1;
        } else {
            const auto min_prc_neighborhood{prcs_neighborhood.minCoeff() + 1};
            prcs[drone_idx] = static_cast<std::size_t>(std::copysign(1u, min_prc_neighborhood)) *
                              std::min(min_prc_neighborhood, nb_drones);
            prcs_neighborhood.setConstant(nb_drones);
        }
    }
}

void WeightedTopologyNeighbors::enrich_neighborhood(WeightedTopologyNeighborsMsg &neighborhood) {
// When we arrive here, all the prcs have been updated
    const auto sum_updated_prcs_neighborhood
            {
                    std::accumulate(std::begin(neighborhood.neighbors_ids), std::end(neighborhood.neighbors_ids),
                                    0u,
                                    [this](const double sum, const auto id) { return sum + prcs[id]; })
            };
    Weights weights;
    weights.reserve(nb_drones);
    weights.assign(nb_drones, 0.);
// TODO: Modify the neighborhood position with the interdistance wanted between each drone
// Directly send what it is needed X,V (for the yaw we can control it directly in the controller with a simple command law)
    std::for_each(std::begin(neighborhood.neighbors_ids), std::end(neighborhood.neighbors_ids),
                  [this, &weights, sum_updated_prcs_neighborhood](const auto id) {
                      weights[id] =
                              static_cast<double>(prcs[id]) / static_cast<double>(sum_updated_prcs_neighborhood);
                  });
    neighborhood.set__weights(weights);
}


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightedTopologyNeighbors>());
    rclcpp::shutdown();
    return 0;
}