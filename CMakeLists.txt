cmake_minimum_required(VERSION 3.22)
project(power-rune)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

find_package(OpenCV REQUIRED)


add_executable(${PROJECT_NAME}
        src/main.cpp
        src/RuneRecognizer/Feature.hpp
        src/RuneRecognizer/Matches.hpp
        src/Converter/ConvertTo3D.hpp
        src/Stabilizer.hpp
        src/prediction/Predictor.hpp
        src/Alias.hpp
)

target_link_libraries(${PROJECT_NAME}
        ${OpenCV_LIBS}
)

# Test
enable_testing()
FetchContent_Declare(googletest
        URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

add_executable(RuneRecognizerTest
        src/RuneRecognizer/Feature.hpp
        src/RuneRecognizer/Matches.hpp
        test/RuneRecognizer/FeatureTest.cpp
        test/RuneRecognizer/MatchesTest.cpp

)
target_link_libraries(RuneRecognizerTest
        ${OpenCV_LIBS}
        GTest::gtest_main
)

gtest_discover_tests(
        RuneRecognizerTest
)
