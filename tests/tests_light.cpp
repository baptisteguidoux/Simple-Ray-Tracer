
#include <gtest/gtest.h>

#include "ray.hpp"
#include "geo.hpp"
#include "tuple.hpp"
#include "light.hpp"


TEST(PointLightTest, Constructor) {

  // A point light has a position and intensity
  auto position = math::Point(0.0, 0.0, 0.0);
  auto intensity = color::Color(1, 1, 1);

  auto light = light::PointLight(position, intensity);
  EXPECT_EQ(light.position, position);
  EXPECT_EQ(light.intensity, intensity);
}

TEST(PointLightTest, Equality) {

  auto position = math::Point(0.0, 0.0, 0.0);
  auto intensity = color::Color(1, 1, 1);
  auto light1 = light::PointLight(position, intensity);
  
  auto position2 = math::Point(0.0, 1.0, 0.0);
  auto intensity2 = color::Color(2, 1, 1);
  auto light2 = light::PointLight(position2, intensity2);

  auto position3 = math::Point(0.0, 0.0, 0.0);
  auto intensity3 = color::Color(1, 1, 1);
  auto light3 = light::PointLight(position3, intensity3);

  EXPECT_EQ(light1, light3);
  EXPECT_NE(light1, light2);
  EXPECT_NE(light3, light2);
}

