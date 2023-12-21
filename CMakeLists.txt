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

add_executable(main
        src/PowerRuneNode.cpp
        src/Alias.hpp
        src/RuneRecognizer/Feature.hpp
        src/RuneRecognizer/Matches.hpp
)
target_include_directories(main PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

ament_target_dependencies(
        main
        "rclcpp"
        "std_msgs"
        "sensor_msgs"
        "geometry_msgs"
        "cv_bridge"
        "OpenCV"
)

install(TARGETS main
        DESTINATION lib/${PROJECT_NAME})
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
