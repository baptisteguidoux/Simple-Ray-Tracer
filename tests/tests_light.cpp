
#include <memory>
#include <cmath>

#include <gtest/gtest.h>

#include "ray.hpp"
#include "geo.hpp"
#include "tuple.hpp"
#include "light.hpp"
#include "color.hpp"
#include "material.hpp"
#include "world.hpp"


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

  auto result = light::lighting(sphere.get(), light, position, eyev, normalv, 1.0);
  EXPECT_EQ(result, color::Color(1.9, 1.9, 1.9));

  // Lighting with the eye between light and surface, eye offset 45 degrees
  // --> intensity = 0.1 (ambient) + 0.9 (diffuse) + 0.0 (specular)  
  eyev = math::Vector(0.0, sqrt(2)/2, sqrt(2)/2);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 0.0, -10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere.get(), light, position, eyev, normalv, 1.0);
  EXPECT_EQ(result, color::Color(1.0, 1.0, 1.0));

  // Lighting with eye opposite surface, light offset 45 degrees
  eyev = math::Vector(0.0, 0.0, -1.0);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 10.0, -10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere.get(), light, position, eyev, normalv, 1.0);
  EXPECT_EQ(result, color::Color(0.7364, 0.7364, 0.7364));

  // Lighting with eye in the path of the reflection vector
  eyev = math::Vector(0.0, -sqrt(2)/2.0, -sqrt(2)/2.0);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 10.0, -10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere.get(), light, position, eyev, normalv, 1.0);
  EXPECT_EQ(result, color::Color(1.6364, 1.6364, 1.6364));

  // Lighting with the light behind the surface
  // total intensity should = ambient
  eyev = math::Vector(0.0, 0.0, -1.0);
  normalv = math::Vector(0.0, 0.0, -1.0);
  light = light::PointLight(math::Point(0.0, 0.0, 10.0), color::Color(1, 1, 1));
  result = light::lighting(sphere.get(), light, position, eyev, normalv, 1.0);
  EXPECT_EQ(result, color::Color(0.1, 0.1, 0.1));
  
  // Lighting with the surface in shadow
  eyev = math::Vector(0, 0, -1);
  normalv = math::Vector(0, 0, -1);
  light = light::PointLight(math::Point(0, 0, -10), color::Color(1, 1, 1));
  float in_shadow = 0.0;
  result = light::lighting(sphere.get(), light, position, eyev, normalv, in_shadow);
  EXPECT_EQ(result, color::Color(0.1, 0.1, 0.1));
}

TEST(LightTest, LightingFunctionUsesLightIntensityToAttenuateColor) {

  auto world = world::build_default_world();
  world.light = light::PointLight(math::Point(0, 0, -10), color::WHITE);
  auto shape = world.objects[0];
  shape->material.ambient = 0.1;
  shape->material.diffuse = 0.9;
  shape->material.specular = 0;
  shape->material.color = color::WHITE;

  math::Point point(0, 0, -1);
  math::Vector eyev(0, 0, -1);
  math::Vector normalv(0, 0, -1);

  EXPECT_EQ(light::lighting(shape.get(), *world.light, point, eyev, normalv, 1.0), color::WHITE);
  EXPECT_EQ(light::lighting(shape.get(), *world.light, point, eyev, normalv, 0.5), color::Color(0.55, 0.55, 0.55));
  EXPECT_EQ(light::lighting(shape.get(), *world.light, point, eyev, normalv, 0.0), color::Color(0.1, 0.1, 0.1));
}

TEST(LightTest, AreaLightConstructor) {

  math::Point corner(0, 0, 0);
  math::Vector v1(2, 0, 0);
  math::Vector v2(0, 0, 1);
  auto light = light::AreaLight(corner, v1, 4, v2, 2, color::WHITE);
  
  EXPECT_EQ(light.corner, corner);
  EXPECT_EQ(light.uvec, math::Vector(0.5, 0, 0));
  EXPECT_EQ(light.usteps, 4);
  EXPECT_EQ(light.vvec, math::Vector(0, 0, 0.5));
  EXPECT_EQ(light.vsteps, 2);
  EXPECT_EQ(light.samples, 8);
  EXPECT_EQ(light.position, math::Point(1, 0, 0.5));
}

TEST(LightTest, FindSinglePointOnAreaLight) {

  struct TestInput {
    uint u;
    uint v;
    math::Tuple point;
    TestInput(const uint u_, const uint v_, const math::Tuple p) : u {u_}, v {v_}, point {p} {}
  };

  std::vector<TestInput> inputs {
      TestInput(0, 0, math::Point(0.25, 0, 0.25)),
      TestInput(1, 0, math::Point(0.75, 0, 0.25)),
      TestInput(0, 1, math::Point(0.25, 0, 0.75)),
      TestInput(2, 0, math::Point(1.25, 0, 0.25)),
      TestInput(3, 1, math::Point(1.75, 0, 0.75)),
  };
  
  auto light = light::AreaLight(math::Point(0, 0, 0), math::Vector(2, 0, 0), 4, math::Vector(0, 0, 1), 2, color::WHITE);

  for (const auto& input : inputs) {
    auto pt = light.point_at(input.u, input.v);
    EXPECT_EQ(pt, input.point);
  }
  
}

TEST(LightTest, AreaLightIntensityFunction) {

  struct TestInput {
    math::Tuple point;
    float result;

    TestInput(const math::Tuple& p, const float r) : point {p}, result {r} {}
  };

  std::vector<TestInput> inputs {
      TestInput(math::Point(0, 0, 2), 0),
      TestInput(math::Point(1, -1, 2), 0.25),
      TestInput(math::Point(1.5, 0, 2), 0.5),
      TestInput(math::Point(1.25, 1.25, 3), 0.75),
      TestInput(math::Point(0, 0, -2), 1),
  };
  
  auto w = world::build_default_world();
  math::Point corner(-0.5, -0.5, -5);
  math::Vector v1(1, 0, 0);
  math::Vector v2(0, 1, 0);
  light::AreaLight light(corner, v1, 2, v2, 2, color::WHITE);

  for (const auto& input : inputs)
    EXPECT_EQ(light.intensity_at(input.point, w), input.result);
}

