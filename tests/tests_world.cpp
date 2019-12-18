#include <memory>
#include <cmath>

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

TEST(WorldTest, ShadeHit) {

  // Shading an intersection
  auto wrld = world::build_default_world();
  auto r = ray::Ray(math::Point(0.0, 0.0, -5.0), math::Vector(0.0, 0.0, 1.0));
  auto shape = wrld.objects.at(0);
  auto ixs = geo::Intersection(4.0, shape);
  
  auto comps = geo::prepare_computations(ixs, r);
  auto col = wrld.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.38066, 0.47583, 0.2855));

  // Shading an intersection from the inside
  wrld.light = light::PointLight(math::Point(0.0, 0.25, 0.0), color::Color(1, 1, 1));
  r = ray::Ray(math::Point(0.0, 0.0, 0.0), math::Vector(0.0, 0.0, 1.0));
  shape = wrld.objects.at(1);
  ixs = geo::Intersection(0.5, shape);

  comps = prepare_computations(ixs, r);
  col = wrld.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.90498, 0.90498, 0.90498));

  // Shading an intersection in shadow
  // sphere 2 is in shadow of sphere 1, so the only reflection is the ambient
  wrld = world::World();
  wrld.light = light::PointLight(math::Point(0, 0, -10), color::Color(1, 1, 1));
  auto s1 = std::make_shared<geo::Sphere>();
  wrld.objects.push_back(s1);
  auto s2 = std::make_shared<geo::Sphere>();
  s2->transform = math::translation(0, 0, 10);
  wrld.objects.push_back(s2);
  r = ray::Ray(math::Point(0, 0, 5), math::Vector(0, 0, 1));
  ixs = geo::Intersection(4, s2);
  comps = prepare_computations(ixs, r);
  col = wrld.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.1, 0.1, 0.1));
}

TEST(WorldTest, ColorAt) {

  // Black when the ray miss
  auto wrld = world::build_default_world();
  auto r = ray::Ray(math::Point(0.0, 0.0, -5.0), math::Vector(0.0, 1.0, 0.0));

  auto col = wrld.color_at(r);
  EXPECT_EQ(col, color::Color(0, 0, 0));

  // The color when a ray hits
  r = ray::Ray(math::Point(0.0, 0.0, -5.0), math::Vector(0.0, 0.0, 1.0));

  col = wrld.color_at(r);
  EXPECT_EQ(col, color::Color(0.38066, 0.47583, 0.2855));

  // The color with an intersection behind the ray
  wrld = world::build_default_world();
  auto& outer = wrld.objects.at(0); // first object
  outer->material.ambient = 1;
   //second object
  auto& inner = wrld.objects.at(1);
  inner->material.ambient = 1;
  r = ray::Ray(math::Point(0.0, 0.0, 0.75), math::Vector(0.0, 0.0, -1.0));

  col = wrld.color_at(r);
  EXPECT_EQ(col, inner->material.color);
}

TEST(WorldTest, IsShadowedFunction) {

  // There is no shadow when nothing is collinear with point and light
  auto w = world::build_default_world();
  auto p = math::Point(0, 10, 0);
  EXPECT_FALSE(w.is_point_shadowed(p));

  // There is shadow when an object is between the point and the light
  p = math::Point(10, -10, 10);
  EXPECT_TRUE(w.is_point_shadowed(p));

  // There is no shadow when an object is behind the light
  p = math::Point(-20, 20, -20);
  EXPECT_FALSE(w.is_point_shadowed(p));

  // The same
  p = math::Point(-2, 2, -2);
  EXPECT_FALSE(w.is_point_shadowed(p));
}

TEST(WorldTest, IsShadowedFunctionMaterialDoesNotCastShadow) {

  auto w = world::World();
  w.light = light::PointLight(math::Point(-10, 5, 0), color::Color(1, 1, 1));
  auto plane = std::make_shared<geo::Plane>();
  plane->transform = math::rotation_z(M_PI / 2);
  w.objects.push_back(plane);
  
  auto point = math::Point(20, 0, 0);
  EXPECT_TRUE(w.is_point_shadowed(point));

  w.objects[0]->material.cast_shadow = false;
  EXPECT_FALSE(w.is_point_shadowed(point));
}

TEST(WorldTest, ReflectedColor) {

  // The reflected color for a non reflective material
  auto w = world::build_default_world();
  // ray starts inside the default world spheres
  auto r = ray::Ray(math::Point(0, 0, 0), math::Vector(0, 0, 1));
  // inner sphere
  auto& shape = w.objects.at(1);
  // guarantee any reflection will have something to reflect...
  shape->material.ambient = 1; 
  auto i = geo::Intersection(1, shape);
  auto comps = geo::prepare_computations(i, r);
  auto col = w.reflected_color(comps);
  // ... but the inner sphere is not reflective
  EXPECT_EQ(col, color::BLACK);

  // The reflected color for a reflective material
  w = world::build_default_world();
  // Add a (semi) reflective plane below the spheres of the default scene
  auto shape2 = std::make_shared<geo::Plane>();
  shape2->material.reflective = 0.5;
  shape2->transform = math::translation(0, -1, 0);
  w.objects.push_back(shape2);
  r = ray::Ray(math::Point(0, 0, -3), math::Vector(0, -sqrt(2) / 2, sqrt(2) / 2));
  i = geo::Intersection(sqrt(2), shape2);
  comps = geo::prepare_computations(i, r);
  col = w.reflected_color(comps);
  EXPECT_EQ(col, color::Color(0.19033, 0.23791, 0.14274));
}

TEST(WorldTest, ShadeHitWithReflect) {

  // Shade hit with a reflective material
  auto w = world::build_default_world();
  // ray starts inside the default world spheres
  // inner sphere
  auto shape = std::make_shared<geo::Plane>();
  shape->material.reflective = 0.5;
  shape->transform = math::translation(0, -1, 0);
  w.objects.push_back(shape);
  auto r = ray::Ray(math::Point(0, 0, -3), math::Vector(0, -sqrt(2) / 2, sqrt(2) / 2));
  auto i = geo::Intersection(sqrt(2), shape);
  auto comps = geo::prepare_computations(i, r);
  auto col = w.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.87676, 0.92434, 0.82917));
}

TEST(WorldTest, ReflectionAvoidInfiniteRecursion) {

  // Handles infinite recursion
  // Place two reflective planes face to face
  auto w = world::World();
  w.light = light::PointLight(math::Point(0, 0, 0), color::Color(1, 1, 1));
  auto lower = std::make_shared<geo::Plane>();
  lower->material.reflective = 1;
  lower->transform = math::translation(0, -1, 0);
  w.objects.push_back(lower);
  auto upper = std::make_shared<geo::Plane>();
  upper->material.reflective = 1;
  upper->transform = math::translation(0, 1, 0);
  w.objects.push_back(upper);
  auto r = ray::Ray(math::Point(0, 0, 0), math::Vector(0, 1, 0));
  // Test no segfault (infinite recursion --> core dumped)
  EXPECT_EXIT((w.color_at(r), exit(0)), ::testing::ExitedWithCode(0), ".*");
}

TEST(WorldTest, ReflectionLimitRecursion) {

  auto w = world::build_default_world();
  // ray starts inside the default world spheres
  // inner sphere
  auto shape = std::make_shared<geo::Plane>();
  shape->material.reflective = 0.5;
  shape->transform = math::translation(0, -1, 0);
  w.objects.push_back(shape);
  auto r = ray::Ray(math::Point(0, 0, -3), math::Vector(0, -sqrt(2) / 2, sqrt(2) / 2));
  auto i = geo::Intersection(sqrt(2), shape);
  auto comps = geo::prepare_computations(i, r);
  auto col = w.reflected_color(comps, 0); //no recursive call remaining
  EXPECT_EQ(col, color::BLACK);
}

