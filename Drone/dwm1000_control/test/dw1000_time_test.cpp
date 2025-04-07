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
 * Subtract of two consecutive timestamps
 */
TEST_F(DW1000TimeTest, Subtract_bytewise) {
    uint8_t ts1[5] = {0xF0, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t ts2[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    DW1000Time t1(ts1);
    DW1000Time t2(ts2);  // max 40-bit value

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
    DW1000Time t2(0xFF); 

    /* substraction operator */
    DW1000Time delta = t2 - t1;
    EXPECT_EQ(delta.get_timestamp(), 0x100);

    /* substraction assignment operator */
    t2 -= t1;
    EXPECT_EQ(t2.get_timestamp(), 0x100);
}   


/**
 * Timer overrun of two consecutive timestamps
 */
TEST_F(DW1000TimeTest, SubtractWithWrap_bytewise) {
    uint8_t ts1[5] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    uint8_t ts2[5] = {0xFF, 0x00, 0x00, 0x00, 0x00};
    DW1000Time t1(ts1);  // max 40-bit value
    DW1000Time t2(ts2); 

    /* substraction operator */
    DW1000Time delta = t2 - t1;
    EXPECT_EQ(delta.get_timestamp(), 0x100);

    /* substraction assignment operator */
    t2 -= t1;
    EXPECT_EQ(t2.get_timestamp(), 0x100);
} 

/**
 * 
 */
TEST_F(DW1000TimeTest, SubtractZero) {
    DW1000Time t1(static_cast<uint64_t>(0x1000));
    DW1000Time t2(static_cast<uint64_t>(0));  // zero timestamp

    /* Subtract zero from t1 */
    DW1000Time delta = t1 - t2;
    EXPECT_EQ(delta.get_timestamp(), 0x1000);
    
    /* check wraparound for negative values */
    delta = t2 - t1;
    EXPECT_EQ(delta.get_timestamp(), 0xFFFFFFF000);

    // Subtraction assignment with zero
    t1 -= t2;
    EXPECT_EQ(t1.get_timestamp(), 0x1000);
    t2 -= t1;
    EXPECT_EQ(t2.get_timestamp(), 0xFFFFFFF000);
}

/**
 * 
 */
TEST_F(DW1000TimeTest, SubtractZero_bytewise) {
    uint8_t ts1[5] = {0x00, 0x10, 0x00, 0x00, 0x00}; // 0x1000
    uint8_t ts2[5] = {0x00, 0x00, 0x00, 0x00, 0x00}; 
    DW1000Time t1(ts1);
    DW1000Time t2(ts2);  // zero timestamp

    /* Subtract zero from t1 */
    DW1000Time delta = t1 - t2;
    EXPECT_EQ(delta.get_timestamp(), 0x1000);
    
    /* check wraparound for negative values */
    delta = t2 - t1;
    EXPECT_EQ(delta.get_timestamp(), 0xFFFFFFF000);

    // Subtraction assignment with zero
    t1 -= t2;
    EXPECT_EQ(t1.get_timestamp(), 0x1000);
    t2 -= t1;
    EXPECT_EQ(t2.get_timestamp(), 0xFFFFFFF000);
}


/**
 * 
 */
//TEST_F(DW1000TimeTest, Addition) {
//    DW1000Time t1();
//    DW1000Time t2();  // zero timestamp
//
//    /* Subtract zero from t1 */
//    DW1000Time delta = t1 - t2;
//    EXPECT_EQ(delta.get_timestamp(), 0x1000);
//    
//    /* check wraparound for negative values */
//    delta = t2 - t1;
//    EXPECT_EQ(delta.get_timestamp(), 0xFFFFFFF000);
//
//    // Subtraction assignment with zero
//    t1 -= t2;
//    EXPECT_EQ(t1.get_timestamp(), 0x1000);
//    t2 -= t1;
//    EXPECT_EQ(t2.get_timestamp(), 0xFFFFFFF000);
//}