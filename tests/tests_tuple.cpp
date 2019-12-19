#include <cmath>

#include <gtest/gtest.h>

#include "tuple.hpp"

// move elsewhere if other util func
TEST(UtilsTest, Map) {

  EXPECT_EQ(math::map(20.0, 0, 100.0, 0.0, 1000.0), 200.0);
  EXPECT_EQ(math::map(20.0, 0.0, 100.0, 0.0, -1000.0), -200.0);
  EXPECT_EQ(math::map(20.0, 0.0, 100.0, -1000.0, 0.0), -800.0);
}

TEST(TupleTest, Subscript) {
  math::Tuple tuple (1, 2, 3, 4);

  ASSERT_EQ(tuple[0], 1);
  ASSERT_EQ(tuple[2], 3);

  tuple[2] = 7;
  
  ASSERT_EQ(tuple[2], 7);
}

TEST(TupleTest, IsPoint) {
  math::Tuple tup_p(1.0, -3.6, 0.0, 1.0);
  math::Tuple tup_np(5.2, 3.9, 1.7, 0.0);

  ASSERT_TRUE(is_point(tup_p));
  ASSERT_FALSE(is_point((tup_np)));
}

TEST(TupleTest, IsVector) {
  math::Tuple tup_v(1.0, -3.6, 0.0, 0.0);
  math::Tuple tup_nv(5.2, 3.9, 1.7, 1.0);

  ASSERT_TRUE(is_vector(tup_v));
  ASSERT_FALSE(is_vector(tup_nv));
}

TEST(TupleTest, IsEqual) {
  math::Tuple tup1(1.0, -3.6, 0.0, 0.0);
  math::Tuple tup2(5.2, 3.9, 1.7, 1.0);
  math::Tuple tup3(1.0, -3.6, 0.0, 0.0);

  ASSERT_EQ(tup1, tup3);
  ASSERT_NE(tup1, tup2);
}

TEST(TupleTest, PointFactoryFunction) {
  auto point1 = math::Point(4.0, -4.0, 3.0);

  ASSERT_EQ(point1, math::Tuple(4.0, -4.0, 3.0, 1.0));
}

TEST(TupleTest, VectorFactoryFunction) {
  auto vector1 = math::Vector(4.0, -4.0, 3.0);

  ASSERT_EQ(vector1, math::Tuple(4.0, -4.0, 3.0, 0.0));
}

TEST(TupleTest, Addition) {
  math::Tuple tup1(3.0, -2.0, 5.0, 1.0);
  math::Tuple tup2(-2.0, 3.0, 1.0, 0.0);

  ASSERT_EQ((tup1 + tup2), math::Tuple(1.0, 1.0, 6.0, 1.0));
}


TEST(TupleTest, Substraction) {
  auto point1 = math::Point(3.0, 2.0, 1.0);
  auto point2 = math::Point(5.0, 6.0, 7.0);
  auto vector1 = math::Vector(3.0, 2.0, 1.0);
  auto vector2 = math::Vector(5.0, 6.0, 7.0);

  ASSERT_EQ((point1 - point2), math::Vector(-2.0, -4.0, -6.0));
  ASSERT_EQ((point1 - vector2), math::Point(-2.0, -4.0, -6.0));
  ASSERT_EQ((vector1 - vector2), math::Vector(-2.0, -4.0, -6.0));
}

TEST(TupleTest, Negation) {
  math::Tuple tup1(1.0, -2.0, 3.0, -4.0);

  ASSERT_EQ(-tup1, math::Tuple(-1.0, 2.0, -3.0, 4.0));
}

TEST(TupleTest, ScalarMultiplication) {
  math::Tuple tup1(1.0, -2.0, 3.0, -4.0);

  ASSERT_EQ((tup1 * 3.5), math::Tuple(3.5, -7.0, 10.5, -14.0));
  ASSERT_EQ((tup1 * 0.5), math::Tuple(0.5, -1.0, 1.5, -2.0));
}

TEST(TupleTest, ScalarDivision) {
  math::Tuple tup1(1.0, -2.0, 3.0, -4.0);

  ASSERT_EQ((tup1 / 2.0), math::Tuple(0.5, -1.0, 1.5, -2.0));
}

TEST(TupleTest, Magnitude) {
  ASSERT_EQ(magnitude(math::Vector(1.0, 0.0, 0.0)), 1.0);
  ASSERT_EQ(magnitude(math::Vector(0.0, 1.0, 0.0)), 1.0);
  ASSERT_EQ(magnitude(math::Vector(0.0, 0.0, 1.0)), 1.0);
  // Have to static cast sqrt to compare the results, otherwise it is false
  ASSERT_EQ(magnitude(math::Vector(1.0, 2.0, 3.0)), static_cast<double>(sqrt(14)));
  ASSERT_EQ(magnitude(math::Vector(-1.0, -2.0, -3.0)), static_cast<double>(sqrt(14)));
}

TEST(TupleTest, Normalization) {
  ASSERT_EQ(normalize(math::Vector(4.0, 0.0, 0.0)), math::Vector(1.0, 0.0, 0.0));
  ASSERT_EQ(normalize(math::Vector(1.0, 2.0, 3.0)), math::Vector(1.0, 2.0, 3.0) / magnitude(math::Vector(1.0, 2.0, 3.0)));
  ASSERT_TRUE(math::almost_equal(math::magnitude(math::normalize(math::Vector(1.0, 2.0, 3.0))), 1.0));
}

TEST(TupleTest, DotProduct) {
  ASSERT_EQ(dot(math::Vector(1.0, 2.0, 3.0), math::Vector(2.0, 3.0, 4.0)), 20.0);
}

TEST(TupleTest, CrossProduct) {
  auto vec1 = math::Vector(1.0, 2.0, 3.0);
  auto vec2 = math::Vector(2.0, 3.0, 4.0);

  ASSERT_EQ(cross(vec1, vec2), math::Vector(-1.0, 2.0, -1.0));
  ASSERT_EQ(cross(vec2, vec1), math::Vector(1.0, -2.0, 1.0));
}

