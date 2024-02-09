/**
 * @brief
 * @author Astier Arthur
*/

#include <rclcpp/rclcpp.hpp>
#include "nearest_neighbors.cpp"
#include <custom_msgs/msg/weighted_topology_neighbors_misc.hpp>

class WeightedTopologyNeighbors : public NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighborsMisc> {
    using WeightedTopologyNeighborsMisc = custom_msgs::msg::WeightedTopologyNeighborsMisc;
    using NearestNeighborsType = NearestNeighbors<WeightedTopologyNeighborsMisc>;
public:
    WeightedTopologyNeighbors() : NearestNeighbors<custom_msgs::msg::WeightedTopologyNeighborsMisc>() {
        this->declare_parameter<std::vector<bool>>("leaders");
        leaders = this->get_parameter("leaders").as_bool_array();

    };

private:
    bool process_neighbor_position(const std::size_t neighbor_idx, const VehicleLocalPosition &position,
                                   const VehicleLocalPosition &neighbor_position,
                                   Neighbors &nearest_neighbors, NeighborsMisc &neighbors_misc) {

        bool is_neighbor{
                NearestNeighbors::is_neighbor(position, neighbor_position,
                                              this->neighbor_distance)};
        if (is_neighbor) {
            nearest_neighbors.neighbors_ids.emplace_back(neighbor_idx);
        }
        return is_neighbor;

    }

private:
    std::vector<bool> leaders;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightedTopologyNeighbors>());
    rclcpp::shutdown();
    return 0;
}