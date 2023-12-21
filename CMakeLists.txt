cmake_minimum_required(VERSION 3.8)
project(power_rune)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(src)
add_executable(Node
        src/Node.cpp
        src/Alias.hpp
        src/RuneRecognizer/Feature.hpp
        src/RuneRecognizer/Matches.hpp
        src/Converter/ConvertTo3D.hpp
        src/prediction/Predictor.hpp
)
target_include_directories(Node PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
        Node
        "rclcpp"
        "std_msgs"
        "sensor_msgs"
        "geometry_msgs"
        "cv_bridge"
        "OpenCV"
)

install(TARGETS Node
        DESTINATION lib/${PROJECT_NAME})

ament_package()
