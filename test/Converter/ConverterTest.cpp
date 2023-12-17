#include "../../src/Converter/Converter.hpp"
#include <gtest/gtest.h>
TEST(AbsoluteDirectionConverterTest, ReturnsOptionalEmptyIfSourceBad) {// Arrange
    AbsoluteDirectionConverter<500> converter;
    for (int i = 0; i < 600; ++i) {
        // ReSharper disable once CppNoDiscardExpression// ReSharper disable once CppNoDiscardExpression
        converter.convertToVehicle({0, 0, 1, 1});
    }

    constexpr Rune<Direction> source{{0, 0}, {10, 10}};// nullptr

    // Act
    const auto result = converter.convertToVehicle(source);

    // Assert
    EXPECT_FALSE(result.has_value());
}

TEST(AbsoluteDirectionConverterTest, ReturnsCorrectConversionWhenSourceGood) {
    // Arrange
    AbsoluteDirectionConverter<50> converter;
    for (int i = 0; i < 60; ++i) {
        // ReSharper disable once CppNoDiscardExpression// ReSharper disable once CppNoDiscardExpression
        converter.convertToVehicle({0, 0, 1, 1});
    }
    const Rune<Direction> source{2, 2, 1, 1};// Replace with appropriate value

    // Act
    const auto result = converter.convertToVehicle(source);

    // Assert
    EXPECT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result.value().target.pitch, 2);
    EXPECT_DOUBLE_EQ(result.value().target.yaw, 2);
    EXPECT_DOUBLE_EQ(result.value().centre.pitch, 1);
    EXPECT_DOUBLE_EQ(result.value().centre.yaw, 1);
}
TEST(AbsoluteDirectionConverterTest, ReturnsCorrectConversionWithRotation) {
    // Arrange
    AbsoluteDirectionConverter<50> converter;
    for (int i = 0; i < 60; ++i) {
        // ReSharper disable once CppNoDiscardExpression// ReSharper disable once CppNoDiscardExpression
        converter.convertToVehicle({0, 0, 1, 1});
    }
    converter.gimbal = Direction{0, 1};
    constexpr Rune<Direction> source{2, 2, 1, 2};// Replace with appropriate value

    // Act
    const auto result = converter.convertToVehicle(source);

    // Assert
    EXPECT_TRUE(result.has_value());
    EXPECT_DOUBLE_EQ(result.value().target.pitch, 2);
    EXPECT_DOUBLE_EQ(result.value().target.yaw, 1);
    EXPECT_DOUBLE_EQ(result.value().centre.pitch, 1);
    EXPECT_DOUBLE_EQ(result.value().centre.yaw, 1);
}