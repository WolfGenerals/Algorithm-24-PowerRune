cmake_minimum_required(VERSION 3.22)
project(power-rune)

set(CMAKE_CXX_STANDARD 17)

include(FetchContent)

find_package(OpenCV REQUIRED)


add_executable(${PROJECT_NAME}
        src/main.cpp
        src/Interfaces.hpp
        src/RuneRecognizer/RuneRecognizer.cpp
        src/RuneRecognizer/RuneRecognizer.hpp
        src/RuneRecognizer/Feature.cpp
        src/RuneRecognizer/Feature.hpp
        src/RuneRecognizer/Matches.cpp
        src/RuneRecognizer/Matches.hpp
        src/Converter/Converter.hpp
        src/prediction/pre_bullet.cpp
        src/prediction/pre_bullet.h
        src/prediction/pre_for_small.cpp
        src/prediction/pre_for_small.h
        src/Stabilizer.hpp
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
        test/RuneRecognizer/RuneRecognizerTest.cpp
        src/Interfaces.hpp
        src/RuneRecognizer/RuneRecognizer.cpp
        src/RuneRecognizer/RuneRecognizer.hpp
        src/RuneRecognizer/Feature.cpp
        src/RuneRecognizer/Feature.hpp
        src/RuneRecognizer/Matches.cpp
        src/RuneRecognizer/Matches.hpp
        test/RuneRecognizer/FeatureTest.cpp
        test/RuneRecognizer/MatchesTest.cpp

)
target_link_libraries(RuneRecognizerTest
        ${OpenCV_LIBS}
        GTest::gtest_main
)

add_executable(ConverterTest
        test/Converter/ConverterTest.cpp
        src/Converter/Converter.hpp
)
target_link_libraries(ConverterTest
        ${OpenCV_LIBS}
        GTest::gtest_main
)

add_executable(StabilizedTest
        test/StabilizerTest.cpp
        src/Stabilizer.hpp
)
target_link_libraries(StabilizedTest
        GTest::gtest_main
)
gtest_discover_tests(
        ConverterTest
        RuneRecognizerTest
        StabilizedTest
)
