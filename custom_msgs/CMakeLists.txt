cmake_minimum_required(VERSION 3.8)
project(custom_msgs)


list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
include_directories(

  ${px4_msgs_INCLUDE_DIRS}  # Add this line for px4_msgs
)
# get all msg files
set(MSGS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/msg")
file(GLOB CUSTOM_MSGS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${MSGS_DIR}/*.msg")

# get all srv files
#set(SRVS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/srv")
#file(GLOB CUSTOM_SRVS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "${SRVS_DIR}/*.srv")

# Generate introspection typesupport for C and C++ and IDL files
rosidl_generate_interfaces(${PROJECT_NAME}
	${CUSTOM_MSGS}
	#${CUSTOM_SRVS}
	DEPENDENCIES builtin_interfaces px4_msgs
	ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()


