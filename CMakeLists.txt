cmake_minimum_required(VERSION 3.8)
project(power_rune)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(detector
        src/PowerRuneNode.cpp
        src/Alias.hpp
        src/RuneRecognizer/Feature.hpp
        src/RuneRecognizer/Matches.hpp
)
include_directories(
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        detector
        rclcpp
        std_msgs
        sensor_msgs
        geometry_msgs
        cv_bridge
        OpenCV
)

add_executable(
        predictor
        src/Predictor.cpp
)
ament_target_dependencies(
        predictor
        rclcpp
        std_msgs
        geometry_msgs
        OpenCV
        tf2
        tf2_ros
        tf2_geometry_msgs
)
install(TARGETS predictor
        DESTINATION lib/${PROJECT_NAME})
install(TARGETS detector
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME})
install(DIRECTORY temp
        DESTINATION lib/${PROJECT_NAME})


add_executable(video
        src/TestVideo.cpp
)
ament_target_dependencies(
        video
        "rclcpp"
        "sensor_msgs"
        "cv_bridge"
        "OpenCV"
)
install(TARGETS video
        DESTINATION lib/${PROJECT_NAME})


ament_package()
