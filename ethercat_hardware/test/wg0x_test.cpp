#include "ethercat_hardware/wg0x.h"
#include <gtest/gtest.h>
#include <iostream>
#include <math.h>

/** 
 * Make sure WG0X::timestampDiff funtion should handle wrap around
 * at edge values for 32bit unsigned values.
 */
TEST(WG0X, timestampDiff)
{
  int32_t diff=1000;
  uint32_t a,b;

  a = 0x1;
  b = a + diff;
  EXPECT_EQ(WG0X::timestampDiff(b, a), diff);
  EXPECT_EQ(WG0X::timestampDiff(a, b), -diff);

  a = 0xFFFFFF00;
  b = a + diff;
  EXPECT_EQ(WG0X::timestampDiff(b, a), diff);
  EXPECT_EQ(WG0X::timestampDiff(a, b), -diff);
}


/** 
 * Make sure WG0X::positionDiff funtion should handle position wrap around
 * at edge values for 32bit signed/unsigned values.
 */
TEST(WG0X, positionDiff)
{
  int32_t diff=1000;
  int32_t a,b;

  a = 0x1;
  b = a + diff;
  EXPECT_EQ(WG0X::positionDiff(b, a), diff);
  EXPECT_EQ(WG0X::positionDiff(a, b), -diff);

  a = 0x7FFFFFF0;
  b = a + diff;
  EXPECT_EQ(WG0X::positionDiff(b, a), diff);
  EXPECT_EQ(WG0X::positionDiff(a, b), -diff);

  a = 0x800000F;
  b = a + diff;
  EXPECT_EQ(WG0X::positionDiff(b, a), diff);
  EXPECT_EQ(WG0X::positionDiff(a, b), -diff);

  a = 0x800000F;
  b = a + diff;
  EXPECT_EQ(WG0X::positionDiff(b, a), diff);
  EXPECT_EQ(WG0X::positionDiff(a, b), -diff);
}


/** 
 * Make sure timediffToDuration handles converting multisecond timediff values to Durations
 */
TEST(WG0X, timediffToDuration)
{
  // 1 microsecond ==>  (0 second + 1000 nanoseconds)
  EXPECT_EQ(WG0X::timediffToDuration(1), ros::Duration(0, 1000));

  // -1 microsecond ==> - ( 0 second + 1000 nanoseconds )
  EXPECT_EQ(WG0X::timediffToDuration(-1), ros::Duration(0, -1000));

  // 1,000,000 microseconds ==> ( 1 second + 0 nanoseconds )
  EXPECT_EQ(WG0X::timediffToDuration(1000000), ros::Duration(1, 0));

  // - 1,000,000 microseconds ==> - ( 1 second + 0 nanoseconds )
  EXPECT_EQ(WG0X::timediffToDuration(-1000000), ros::Duration(-1, 0));

  // 1,000,001 microseconds ==> ( 1 second + 1 nanoseconds )
  EXPECT_EQ(WG0X::timediffToDuration(1000001), ros::Duration(1, 1000));

  // - 1,000,001 microseconds ==> - ( 1 second + 1 nanoseconds )
  EXPECT_EQ(WG0X::timediffToDuration(-1000001), ros::Duration(-1, -1000));
}


/** 
 * Make sure calcEncoderVelocity handles wrapped position and timestamp values
 */
TEST(WG0X, calcEncoderVelocity)
{
  int timediff; 
  int positiondiff;
  uint32_t t1,t2;
  int32_t p1,p2;

  // A position change of zero should always provide a velocity of zero (except with timechange is zero)
  t1=0; t2=1;
  p1=0; p2=0;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  0.0);
  
  // A position change of 1 tick over 1 second
  t1=0; t2=1000000;
  p1=0; p2=1;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t2, p2, t1),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t1, p1, t2),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t1, p2, t2),  1.0);

  // A position change of 1 tick over 1 second with different starting times
  // Velocity should equal 1 ticks/second
  p1=0; p2=1;
  timediff = 1000000;

  t1=0; 
  t2=t1+timediff;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t2, p2, t1),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t1, p1, t2),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t1, p2, t2),  1.0);

  t1=0xFFFFfff0; 
  t2=t1+timediff;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t2, p2, t1),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t1, p1, t2),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t1, p2, t2),  1.0);

  t1=0x7FFFfff0; 
  t2=t1+timediff;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t2, p2, t1),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t1, p1, t2),  -1.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t1, p2, t2),  1.0);


  // A position change of 1000 ticks over 1 second with different starting positions
  // Velocity should equal 1000 ticks/second
  positiondiff = 1000;
  t1=0; t2=1000000;

  p1=0; 
  p2=p1+positiondiff;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t2, p2, t1),  -1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t1, p1, t2),  -1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t1, p2, t2),  1000.0);

  p1=0x7FFFFFF0; 
  p2=p1+positiondiff;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t2, p2, t1),  -1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t1, p1, t2),  -1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t1, p2, t2),  1000.0);

  p1=0x8000000F;
  p2=p1+positiondiff;
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t2, p1, t1),  1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t2, p2, t1),  -1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p2, t1, p1, t2),  -1000.0);
  EXPECT_EQ( WG0X::calcEncoderVelocity(p1, t1, p2, t2),  1000.0);
}



// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
