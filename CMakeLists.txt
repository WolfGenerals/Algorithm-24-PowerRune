cmake_minimum_required(VERSION 3.22)
project(power-rune)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

find_package(OpenCV REQUIRED)


include_directories(
        include
)
add_executable(${PROJECT_NAME}
        src/main.cpp
        src/Interfaces.hpp
        src/Alias.hpp
        src/detector/Arm.cpp
        src/detector/Arm.hpp
        src/detector/FeatureMatch.cpp
        src/detector/FeatureMatch.hpp
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
)

# Test
enable_testing()
FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(DetectorTest
        test/DetectorTest.cpp
        src/detector/Arm.cpp
        src/detector/Arm.hpp
        src/detector/FeatureMatch.cpp
        src/detector/FeatureMatch.hpp
)
target_link_libraries(
        DetectorTest
        ${OpenCV_LIBS}
        GTest::gtest_main
)
gtest_discover_tests(DetectorTest)
