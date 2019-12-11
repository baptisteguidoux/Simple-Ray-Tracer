#include <gtest/gtest.h>

#include "color.hpp"
#include "tuple.hpp"


TEST(ColorTest, Members) {
  color::Color color(-0.5, 0.4, 1.7);

  ASSERT_TRUE(math::almost_equal(color.red, -0.5f));
  ASSERT_TRUE(math::almost_equal(color.green, 0.4f));
  ASSERT_TRUE(math::almost_equal(color.blue, 1.7f));
}

TEST(ColorTest, Equality) {
  color::Color color(0.9, 0.6, 0.75);
  ASSERT_EQ(color, color::Color(0.4 + 0.5, 0.05 + 0.55, 0.75));
}

TEST(ColorTest, Unequality) {
  color::Color color(0.9, 0.6, 0.75);
  ASSERT_NE(color, color::Color(0.4, 0.05, 0.4));
}

TEST(ColorTest, Addition) {
  color::Color color1(0.9, 0.6, 0.75);
  color::Color color2(0.7, 0.1, 0.25);
  ASSERT_TRUE((color1 + color2) == color::Color(1.6, 0.7, 1.0));
}

TEST(ColorTest, Substraction) {
  color::Color color1(0.9, 0.6, 0.75);
  color::Color color2(0.7, 0.1, 0.25);
  ASSERT_TRUE((color1 - color2) == color::Color(0.2, 0.5, 0.5));
}

TEST(ColorTest, ScalarSubstraction) {
  color::Color col(0.8, 0.2, 0.4);
  auto res = 1 - col;
  EXPECT_EQ(res, color::Color(0.2, 0.8, 0.6));
}

TEST(ColorTest, ScalarMult) {
  color::Color color(0.2, 0.3, 0.4);
  ASSERT_TRUE(color * 2 == color::Color(0.4, 0.6, 0.8));
}

TEST(ColorTest, ColorMult) {
  color::Color color1(1, 0.2, 0.4);
  color::Color color2(0.9, 1, 0.1);
  ASSERT_TRUE(color1 * color2 == color::Color(0.9, 0.2, 0.04));
}

TEST(ColorTest, BlackAndWhite) {

  EXPECT_EQ(color::BLACK, color::Color(0, 0, 0));
  EXPECT_EQ(color::WHITE, color::Color(1, 1, 1));
}

