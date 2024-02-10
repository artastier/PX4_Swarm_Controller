/**
 * @brief Traits namespace containing traits for neighbor positions attributes.
 * @author Arthur Astier
 */
#pragma once

#include <type_traits>
#include <vector>
#include "px4_msgs/msg/vehicle_local_position.hpp"

namespace traits {

    /**
     * @brief Trait to check if a type has a neighbors_position attribute of type vector<px4_msgs::msg::VehicleLocalPosition>.
     * @details This trait evaluates to true if the type T has a neighbors_position attribute of type vector<px4_msgs::msg::VehicleLocalPosition>, otherwise false.
     * @tparam T The type to check.
     */
    template<typename T, typename = void>
    struct has_neighbors_position_attribute_and_is_VLP : public std::false_type {
    };

    /**
     * @brief Specialization of has_neighbors_position_attribute_and_is_VLP for types having neighbors_position attribute of type vector<px4_msgs::msg::VehicleLocalPosition>.
     * @tparam T The type to check.
     */
    template<typename T>
    struct has_neighbors_position_attribute_and_is_VLP<T, std::void_t<decltype(std::declval<T>().neighbors_position)>>
            : public std::is_same<std::vector<px4_msgs::msg::VehicleLocalPosition>, decltype(std::declval<T>().neighbors_position)> {
    };

    /**
     * @brief Convenience variable template to check if a type has a neighbors_position attribute of type vector<px4_msgs::msg::VehicleLocalPosition>.
     * @tparam T The type to check.
     */
    template<typename T>
    constexpr static auto has_neighbors_position_attribute_and_is_VLP_v = has_neighbors_position_attribute_and_is_VLP<T>::value;

    /**
     * @brief Template struct to check if a type has a shared pointer alias.
     * @tparam T The type to check.
     * @tparam void SFINAE parameter for template specialization.
     */
    template<typename T, typename = void>
    struct has_shared_ptr : std::false_type {};

    /**
     * @brief Template struct specialization to check if a type has a shared pointer alias.
     * @tparam T The type to check.
     */
    template<typename T>
    struct has_shared_ptr<T, std::void_t<typename T::SharedPtr>> : std::true_type {};

    /**
     * @brief Helper variable template to check if a type has a shared pointer alias.
     * @tparam T The type to check.
     */
    template<typename T>
    constexpr static auto has_shared_ptr_v= has_shared_ptr<T>::value;


}
