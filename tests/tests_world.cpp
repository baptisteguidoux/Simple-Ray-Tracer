#include <memory>

#include <gtest/gtest.h>

#include "world.hpp"
#include "color.hpp"
#include "tuple.hpp"
#include "matrix.hpp"
#include "geo.hpp"
#include "ray.hpp"


TEST(WorldTest, Constructor) {

  // Create an empty world
  auto world = world::World();
  EXPECT_TRUE(world.objects.size() ==  0);
  // world.light is a std::optional
  EXPECT_EQ(world.light, std::nullopt);
}

TEST(WorldTest, ContainsObjectFunc) {

  auto world = world::World();

  auto sphere = geo::Sphere();
  
  world.objects.push_back(std::make_shared<geo::Sphere>(sphere));

  EXPECT_TRUE(world.contains_object(std::make_shared<geo::Sphere>(sphere)));

  sphere = geo::Sphere();
  sphere.material.ambient = 1;
  
  EXPECT_FALSE(world.contains_object(std::make_shared<geo::Sphere>(sphere)));  
}

TEST(WorldTest, DefaultConstructor) {

  // The default world
  auto light = light::PointLight(math::Point(-10.0, 10.0, -10.0), color::Color(1, 1, 1));

  auto sphere1 = std::make_shared<geo::Sphere>();
  sphere1->material.color = color::Color(0.8, 1.0, 0.6);
  sphere1->material.diffuse = 0.7;
  sphere1->material.specular = 0.2;

  auto sphere2 = std::make_shared<geo::Sphere>();
  sphere2->transform = math::scaling(0.5, 0.5, 0.5);

  auto default_world = world::build_default_world();
  EXPECT_EQ(*default_world.light, light); // access optional
  EXPECT_TRUE(default_world.contains_object(sphere1));
  EXPECT_TRUE(default_world.contains_object(sphere2));
}

TEST(WorldTest, WorldIntersects) {

  // Interwsect a world with a ray
  auto default_world = world::build_default_world();
  auto r = ray::Ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));

  auto xs = default_world.intersects(r);

  EXPECT_EQ(xs.size(), 4);
  EXPECT_EQ(xs[0].t, 4);
  EXPECT_EQ(xs[1].t, 4.5);
  EXPECT_EQ(xs[2].t, 5.5);
  EXPECT_EQ(xs[3].t, 6);
}

