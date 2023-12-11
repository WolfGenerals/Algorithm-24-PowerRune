cmake_minimum_required(VERSION 3.22)
project(power-rune)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

FetchContent_Declare(yml
        GIT_REPOSITORY https://github.moeyy.xyz/https://github.com/jbeder/yaml-cpp
        #        GIT_SHALLOW FALSE  # ensure submodules are checked out
)
FetchContent_MakeAvailable(yml)

find_package(OpenCV REQUIRED)


add_executable(${PROJECT_NAME}
        src/main.cpp
        src/Interfaces.hpp
        src/detector/api.cpp
        src/detector/api.hpp
        src/detector/Feature.cpp
        src/detector/Feature.hpp
        src/detector/Matches.cpp
        src/detector/Matches.hpp
        src/filter/Converter.cpp
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
        YAML_CPP
)

# Test
enable_testing()
FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(DetectorTest
        test/DetectorTest.cpp
        src/Interfaces.hpp
        src/detector/api.cpp
        src/detector/api.hpp
        src/detector/Feature.cpp
        src/detector/Feature.hpp
        src/detector/Matches.cpp
        src/detector/Matches.hpp
)
target_link_libraries(
        DetectorTest
        ${OpenCV_LIBS}
        GTest::gtest_main
)
gtest_discover_tests(DetectorTest)
