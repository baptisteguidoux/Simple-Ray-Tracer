
#include <memory>
#include <cmath>

#include <gtest/gtest.h>

#include "ray.hpp"
#include "geo.hpp"
#include "tuple.hpp"
#include "light.hpp"
#include "color.hpp"
#include "material.hpp"


TEST(LightTest, PointLightConstructor) {

  // A point light has a position and intensity
  auto position = math::Point(0.0, 0.0, 0.0);
  auto intensity = color::Color(1, 1, 1);

  auto light = light::PointLight(position, intensity);
  EXPECT_EQ(light.position, position);
  EXPECT_EQ(light.intensity, intensity);

  // Default PointLight
  auto light2 = light::PointLight();
  EXPECT_EQ(light2.position, math::Point(0, 0, 0));
  EXPECT_EQ(light2.intensity, color::Color(0, 0, 0));
}

TEST(LightTest, PointLightEquality) {

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

TEST(LightTest, LightingFunction) {

  auto sphere = std::make_shared<geo::Sphere>();
  auto mat = material::Material();
  sphere->material = mat;
  auto position = math::Point(0.0, 0.0, 0.0);

  // Lighting with the eye between the light and surface
  // --> intensity = 0.1 (ambient) + 0.9 (diffuse) + 0.9 (specular)
  auto eyev = math::Vector(0.0, 0.0, -1.0);
  auto normalv = math::Vector(0.0, 0.0, -1.0);
  auto light = light::PointLight(math::Point(0.0, 0.0, -10.0), color::Color(1, 1, 1));

  auto result = light::lighting(sphere, light, position, eyev, normalv, false);
  EXPECT_EQ(result, color::Color(1.9, 1.9, 1.9));

  // Lighting with the eye between light and surface, eye offset 45 degrees
  // --> intensity = 0.1 (ambient) + 0.9 (diffuse) + 0.0 (specular)  
  eyev = math::Vector(0.0, sqrt(2)/2, sqrt(2)/2);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 0.0, -10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere, light, position, eyev, normalv, false);
  EXPECT_EQ(result, color::Color(1.0, 1.0, 1.0));

  // Lighting with eye opposite surface, light offset 45 degrees
  eyev = math::Vector(0.0, 0.0, -1.0);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 10.0, -10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere, light, position, eyev, normalv, false);
  EXPECT_EQ(result, color::Color(0.7364, 0.7364, 0.7364));

  // Lighting with eye in the path of the reflection vector
  eyev = math::Vector(0.0, -sqrt(2)/2.0, -sqrt(2)/2.0);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 10.0, -10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere, light, position, eyev, normalv, false);
  EXPECT_EQ(result, color::Color(1.6364, 1.6364, 1.6364));

  // Lighting with the light behind the surface
  // total intensity should = ambient
  eyev = math::Vector(0.0, 0.0, -1.0);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 0.0, 10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere, light, position, eyev, normalv, false);
  EXPECT_EQ(result, color::Color(0.1, 0.1, 0.1));
  
  // Lighting with the surface in shadow
  eyev = math::Vector(0, 0, -1);
  normalv = math::Vector(0, 0, -1);
  light = light::PointLight(math::Point(0, 0, -10), color::Color(1, 1, 1));
  bool in_shadow = true;
  result = light::lighting(sphere, light, position, eyev, normalv, in_shadow);
  EXPECT_EQ(result, color::Color(0.1, 0.1, 0.1));
}

