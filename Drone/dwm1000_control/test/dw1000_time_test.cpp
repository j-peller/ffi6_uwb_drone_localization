#include <gtest/gtest.h>
#include <stdint.h>
#include "../inc/dw1000_time.hpp"

// Test Fixture for DW1000Time
class DW1000TimeTest : public ::testing::Test {
protected:
    // SetUp() is called before each test
    void SetUp() override {
    }

    // TearDown() is called after each test
    void TearDown() override {
    }
};

/**
 * Subtract of two consecutive timestamps
 */
TEST_F(DW1000TimeTest, Subtract) {
    DW1000Time t1(0xFFFFFFFFF0);
    DW1000Time t2(0xFFFFFFFFFF);  // max 40-bit value

    /* substraction operator */
    DW1000Time delta = t2 - t1;
    EXPECT_EQ(delta.get_timestamp(), 0x0F);

    /* substraction assignment operator */
    t2 -= t1;
    EXPECT_EQ(t2.get_timestamp(), 0x0F);
}  

/**
 * Timer overrun of two consecutive timestamps
 */
TEST_F(DW1000TimeTest, SubtractWithWrap) {
    DW1000Time t1(0xFFFFFFFFFF);  // max 40-bit value
    DW1000Time t2(0x10);

    /* substraction operator */
    DW1000Time delta = t2 - t1;
    EXPECT_EQ(delta.get_timestamp(), 0x11);

    /* substraction assignment operator */
    t2 -= t1;
    EXPECT_EQ(t2.get_timestamp(), 0x11);
}   

/**
 * 
 */
TEST_F(DW1000TimeTest, SubtractZero) {
    DW1000Time t1(static_cast<uint64_t>(1000));
    DW1000Time t2(static_cast<uint64_t>(0));  // zero timestamp

    /* Subtract zero from t1 */
    DW1000Time delta = t2 - t1;
    EXPECT_EQ(delta.get_timestamp(), 0);
    
    /* check wraparound for negative values */
    //delta = t2 - t1;
    //EXPECT_EQ(delta.get_timestamp(), 0);

    // Subtraction assignment with zero
    t1 -= t2;
    EXPECT_EQ(t1.get_timestamp(), 1000);
}
