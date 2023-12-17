#include "../src/Stabilizer.hpp"
#include <gtest/gtest.h>

TEST(StabilizerDoubleTest, OneData) {
    StabilizedDouble<1> stabilizer;
    stabilizer = 1.0;
    ASSERT_EQ(stabilizer, 1.0);
}

TEST(StabilizerDoubleTest, TwoData) {
    StabilizedDouble<10> stabilizer;
    stabilizer = 1.0;
    stabilizer = 2.0;
    ASSERT_EQ(stabilizer, 1.5);
}

TEST(StabilizerDoubleTest, FullData) {
    StabilizedDouble<10> stabilizer;
    for (int i = 0; i < 20; ++i) {
        stabilizer = i;
    }
    ASSERT_EQ(stabilizer, 14.5);
}

TEST(StabilizerDoubleTest, ErrorData) {
    StabilizedDouble<50> stabilizer;
    for (int i = 0; i < 150; ++i) {
        stabilizer = 5;
    }
    stabilizer = 10;
    ASSERT_EQ(stabilizer, 5);
}