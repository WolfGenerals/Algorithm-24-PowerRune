#include "../src/Data.hpp"
#include "gtest/gtest.h"

TEST(AngleTest, New) {
    Angle a{1};
    EXPECT_DOUBLE_EQ(a.radian, 1);
    Angle b{1+M_PI};
    EXPECT_DOUBLE_EQ(b.radian, M_PI + 1);
    Angle c{1+2*M_PI};
    EXPECT_DOUBLE_EQ(c.radian, 1);
    Angle d{1-2*M_PI};
    EXPECT_DOUBLE_EQ(d.radian, 1);
}

TEST(DirectionTest, New) {
    Direction a{1, 2};
    EXPECT_DOUBLE_EQ(a.pitch, 1);
    EXPECT_DOUBLE_EQ(a.yaw, 2);
    Direction b{a};
    EXPECT_DOUBLE_EQ(b.pitch, 1);
    EXPECT_DOUBLE_EQ(b.yaw, 2);
    auto c = Direction{3, 4};
    EXPECT_DOUBLE_EQ(c.pitch, 3);
    EXPECT_DOUBLE_EQ(c.yaw, Angle{4});
    Direction d{};
    EXPECT_DOUBLE_EQ(d.pitch, 0);
    EXPECT_DOUBLE_EQ(d.yaw, 0);
}
TEST(DirectionTest, Add) {
    Direction a{1, 2};
    Direction b{3, 4};
    Direction c = a + b;
    EXPECT_DOUBLE_EQ(c.pitch, Angle{4});
    EXPECT_DOUBLE_EQ(c.yaw, Angle{6});
}

TEST(DirectionTest, Sub) {
    Direction a{1, 2};
    Direction b{3, 4};
    Direction c = a - b;
    EXPECT_DOUBLE_EQ(c.pitch, -2);
    EXPECT_DOUBLE_EQ(c.yaw, -2);
}

TEST(DirectionTest, Angle) {
    Direction a{1, 0.5};
    EXPECT_DOUBLE_EQ(a.angle(), 1.0767867445664643);
    Direction b{0, 0};
    EXPECT_DOUBLE_EQ(b.angle(), 0.0);
    Direction c{-1, 0};
    EXPECT_DOUBLE_EQ(c.angle(), 1);
    Direction d{0, -1};
    EXPECT_DOUBLE_EQ(d.angle(), 1);
    Direction e{0, 1};
    EXPECT_DOUBLE_EQ(e.angle(), 1);
    Direction f{1, 1};
    EXPECT_DOUBLE_EQ(f.angle(), 1.2745557823062943);
}

TEST(PlaneTest, New) {
    Plane a{1, 2};
    EXPECT_DOUBLE_EQ(a.distanse, 1);
    EXPECT_DOUBLE_EQ(a.yaw, 2);
    Plane b{a};
    EXPECT_DOUBLE_EQ(b.distanse, 1);
    EXPECT_DOUBLE_EQ(b.yaw, 2);
    auto c = Plane{3, 4};
    EXPECT_DOUBLE_EQ(c.distanse, 3);
    EXPECT_DOUBLE_EQ(c.yaw, Angle{4});
    Plane d{};
    EXPECT_DOUBLE_EQ(d.distanse, 0);
    EXPECT_DOUBLE_EQ(d.yaw, 0);
}
TEST(PlaneTest, Intersect) {
    Plane       a{1, 1};
    Direction   b{M_PI_4, 1 + M_PI_4};
    cv::Point2d c = a.intersect(b);
    EXPECT_DOUBLE_EQ(c.x, -1);
    EXPECT_DOUBLE_EQ(c.y, sqrt(2));
}
TEST(PlaneTest, Direction) {
    Plane       a{1, 1};
    cv::Point2d b{-1, sqrt(2)};
    Direction   c = a.direction(b);
    EXPECT_DOUBLE_EQ(c.pitch, M_PI_4);
    EXPECT_DOUBLE_EQ(c.yaw, 1 + M_PI_4);
}