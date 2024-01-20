cmake_minimum_required(VERSION 3.8)
project(px4_swarm_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include ${YAML_CPP_INCLUDE_DIRS})
add_executable(leader_control src/offboard_control.cpp
)
ament_target_dependencies(leader_control rclcpp px4_msgs)

add_executable(waypoint src/waypoint.cpp)
ament_target_dependencies(waypoint rclcpp px4_msgs std_msgs)
target_link_libraries(waypoint ${YAML_CPP_LIBRARIES})

install(TARGETS
        leader_control waypoint
        DESTINATION lib/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

# Install Py Packages
ament_python_install_package(${PROJECT_NAME})

# Install Py executables
install(PROGRAMS ${PROJECT_NAME}/simulation_node.py DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
        DESTINATION share/${PROJECT_NAME})
ament_package()
