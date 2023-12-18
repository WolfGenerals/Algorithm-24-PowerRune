#include "../../src/Converter/FitPlane.hpp"
#include "../../src/Converter/PixelToDirection.hpp"
#include <gtest/gtest.h>
TEST(AbsoluteDirectionConverterTest, ReturnsOptionalEmptyIfSourceBad) {// Arrange
    PixelToDirection converter{100, 1024, 1024, 100, 1};
    for (int i = 0; i < 100; ++i) {
        // ReSharper disable once CppNoDiscardExpression// ReSharper disable once CppNoDiscardExpression
        converter({{0, 0}, {1, 1}});
    }

    const Rune<cv::Point2d> source{{0, 0}, {512, 512}};

    // Act
    const auto result = converter(source);

    // Assert
    EXPECT_FALSE(result.has_value());
}

TEST(AbsoluteDirectionConverterTest, ReturnsCorrectConversionWhenSourceGood) {
    // Arrange
    PixelToDirection converter{500, 1024, 1024, 100, 1};
    for (int i = 0; i < 60; ++i) {
        // ReSharper disable once CppNoDiscardExpression// ReSharper disable once CppNoDiscardExpression
        converter({{0, 0}, {1, 1}});
    }
    const Rune<cv::Point2d> source{{0, 0}, {1, 1}};// Replace with appropriate value

    // Act
    const auto result = converter(source);

    // Assert
    EXPECT_TRUE(result.has_value());
}

TEST(AbsoluteDirectionConverterTest, ReturnsCorrectConversionWithRotation) {
    // Arrange
    PixelToDirection converter{500, 1024, 1024, 100, 1};
    for (int i = 0; i < 60; ++i) {
        // ReSharper disable once CppNoDiscardExpression// ReSharper disable once CppNoDiscardExpression
        converter({{0, 0}, {1, 1}});
    }
    converter.gimbal = Direction{0, 1};
    const Rune<cv::Point2d> source{{2, 2}, {1, 2}};// Replace with appropriate value

    // Act
    const auto result = converter(source);
    // Assert
    EXPECT_TRUE(result.has_value());
}

TEST(FitPlaneTest, ReturnsCorrectPlane) {
    constexpr double      distance = 100;
    constexpr double      radius   = 10;
    constexpr double      yaw_     = 5;
    FitPlane              fitPlane{radius, 100, 1};
    const Rune<Direction> rune{Direction{0, std::atan2(radius, distance) + yaw_}, Direction{0, yaw_}};
    const auto [distanse, yaw] = fitPlane(rune);
    EXPECT_FLOAT_EQ(distanse, distance);
    EXPECT_FLOAT_EQ(yaw, 5);
}
