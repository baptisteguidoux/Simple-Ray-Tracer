
#include <gtest/gtest.h>

#include "ray.hpp"
#include "tuple.hpp"
#include "matrix.hpp"


TEST(RayTest, Constructor) {

  auto origin = math::Point(1, 2, 3);
  auto direction = math::Vector(4, 5, 6);

  ray::Ray ray(origin, direction);

  EXPECT_EQ(ray.origin, origin);
  EXPECT_EQ(ray.direction, direction);
}

TEST(RayTest, ComputePoint) {

  ray::Ray ray(math::Point(2, 3, 4), math::Vector(1, 0, 0));

  // Find the position of ray after x unit of time
  EXPECT_EQ(ray.position(0), math::Point(2, 3, 4));
  EXPECT_EQ(ray.position(1), math::Point(3, 3, 4));
  EXPECT_EQ(ray.position(-1), math::Point(1, 3, 4));
  EXPECT_EQ(ray.position(2.5), math::Point(4.5, 3, 4));
}

TEST(RayTest, RayTransform) {

  // Translating
  ray::Ray ray(math::Point(1, 2, 3), math::Vector(0, 1, 0));
  auto matrix = math::translation(3, 4, 5);

  auto ray2 = ray.transform(matrix);
  EXPECT_EQ(ray2.origin, math::Point(4, 6, 8));
  EXPECT_EQ(ray2.direction, math::Vector(0, 1, 0));

  // Scaling
  auto matrix2 = math::scaling(2, 3, 4);

  auto ray3 = ray.transform(matrix2);
  EXPECT_EQ(ray3.origin, math::Point(2, 6, 12));
  EXPECT_EQ(ray3.direction, math::Vector(0, 3, 0)); // The direction vector is left unnormalized to compute the proper t value
}

TEST(RayTest, RayEquality) {
  ray::Ray ray1(math::Point(0, 0, 0), math::Vector(0, 0, 1));
  ray::Ray ray2(math::Point(2, 3, 4), math::Vector(0, 0, 1));
  ray::Ray ray3(math::Point(2, 3, 4), math::Vector(1, 0, 0));
  ray::Ray ray4(math::Point(0, 0, 0), math::Vector(1, 0, 0));
  ray::Ray ray5(math::Point(0, 0, 0), math::Vector(0, 0, 1));

  EXPECT_NE(ray1, ray2);
  EXPECT_NE(ray1, ray3);
  EXPECT_NE(ray1, ray4);
  EXPECT_EQ(ray1, ray5);
}

