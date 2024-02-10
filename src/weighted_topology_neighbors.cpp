/**
 * @brief
 * @author Astier Arthur
*/

#include <rclcpp/rclcpp.hpp>
#include "nearest_neighbors.cpp"
#include <custom_msgs/msg/weighted_topology_neighbors.hpp>
#include <eigen3/Eigen/Eigen>

class WeightedTopologyNeighbors : public NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors> {
    using WeightedTopologyNeighborsMsg = custom_msgs::msg::WeightedTopologyNeighbors;
    using NearestNeighborsType = NearestNeighbors<WeightedTopologyNeighbors>;
    using PRCS = Eigen::Vector<std::size_t, Eigen::Dynamic>;
    using Weights = Eigen::VectorXd;

public:
    WeightedTopologyNeighbors() : NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighbors>() {
        this->declare_parameter<std::vector<bool>>("leaders");
        leaders = this->get_parameter("leaders").as_bool_array();

        prcs = PRCS::Zero(nb_drones);
        prcs_neighborhood = PRCS::Constant(nb_drones, nb_drones);
    };

private:
    void process_neighbor_position(const std::size_t neighbor_idx, const VehicleLocalPosition &position,
                                   const VehicleLocalPosition &neighbor_position,
                                   WeightedTopologyNeighborsMsg &neighborhood) override {

        neighborhood.neighbors_ids.emplace_back(neighbor_idx);
        prcs_neighborhood[neighbor_idx] = prcs[neighbor_idx];
    }

    void process_neighborhood(const std::size_t drone_idx, WeightedTopologyNeighborsMsg &neighborhood) override {
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

    void enrich_neighborhood(WeightedTopologyNeighborsMsg &neighborhood) override {
        // When we arrive here, all the prcs have been updated
        const auto sum_updated_prcs_neighborhood
                {
                        std::accumulate(std::begin(neighborhood.neighbors_ids), std::end(neighborhood.neighbors_ids),
                                        0u,
                                        [this](const double sum, const auto id) { return sum + prcs[id]; })
                };
        Weights weights{Weights::Zero(nb_drones)};
        std::for_each(std::begin(neighborhood.neighbors_ids), std::end(neighborhood.neighbors_ids),
                      [this, &weights, sum_updated_prcs_neighborhood](const auto id) {
                          weights[id] =
                                  static_cast<double>(prcs[id]) / static_cast<double>(sum_updated_prcs_neighborhood);
                      });
        // TODO: Update neighborhood with the weights

    }

private:
    std::vector<bool> leaders;
    PRCS prcs;
    PRCS prcs_neighborhood;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightedTopologyNeighbors>());
    rclcpp::shutdown();
    return 0;
}