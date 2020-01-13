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

  EXPECT_TRUE(world.contains_object(std::make_shared<geo::Sphere>(sphere).get()));

  sphere = geo::Sphere();
  sphere.material.ambient = 1;
  
  EXPECT_FALSE(world.contains_object(std::make_shared<geo::Sphere>(sphere).get()));  
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
  EXPECT_TRUE(default_world.contains_object(sphere1.get()));
  EXPECT_TRUE(default_world.contains_object(sphere2.get()));
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
  auto ixs = geo::Intersection(4.0, shape.get());
  
  auto comps = geo::prepare_computations(ixs, r);
  auto col = wrld.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.38066, 0.47583, 0.2855));

  // Shading an intersection from the inside
  wrld.light = light::PointLight(math::Point(0.0, 0.25, 0.0), color::Color(1, 1, 1));
  r = ray::Ray(math::Point(0.0, 0.0, 0.0), math::Vector(0.0, 0.0, 1.0));
  shape = wrld.objects.at(1);
  ixs = geo::Intersection(0.5, shape.get());

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
  ixs = geo::Intersection(4, s2.get());
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
  const auto& light_position = w.light->position;
  auto p = math::Point(0, 10, 0);
  EXPECT_FALSE(w.is_shadowed(light_position, p));

  // There is shadow when an object is between the point and the light
  p = math::Point(10, -10, 10);
  EXPECT_TRUE(w.is_shadowed(light_position, p));

  // There is no shadow when an object is behind the light
  p = math::Point(-20, 20, -20);
  EXPECT_FALSE(w.is_shadowed(light_position, p));

  // The same
  p = math::Point(-2, 2, -2);
  EXPECT_FALSE(w.is_shadowed(light_position, p));
}

TEST(WorldTest, IsShadowedForOcclusionBetweenTwoPoints) {

  auto world = world::build_default_world();
  math::Point light_position(-10, -10, -10);

  EXPECT_EQ(world.is_shadowed(light_position, math::Point(-10, -10, 10)), false);
  EXPECT_EQ(world.is_shadowed(light_position, math::Point(10, 10, 10)), true);
  EXPECT_EQ(world.is_shadowed(light_position, math::Point(-20, -20, -20)), false);
  EXPECT_EQ(world.is_shadowed(light_position, math::Point(-5, -5, -5)), false);  
}

TEST(WorldTest, EvaluatesIntensityAtGivenPoint) {

  struct TestInput {

    math::Tuple point;
    float result;

    TestInput(const math::Tuple p, const float r) : point {p}, result {r} {}
  };

  std::vector<TestInput> test_inputs {
				      TestInput(math::Point(0, 1.0001, 0), 1.0),
				      TestInput(math::Point(-1.0001, 0, 0), 1.0),
				      TestInput(math::Point(0, 0, -1.0001), 1.0),
				      TestInput(math::Point(0, 0, 1.0001), 0.0),
				      TestInput(math::Point(1.0001, 0, 0), 0.0),
				      TestInput(math::Point(0, -1.0001, 0), 0.0),
				      TestInput(math::Point(0, 0, 0), 0.0)
  };
  
  auto world = world::build_default_world();
  const auto& light= world.light;

  for (const auto& input : test_inputs) {
    float intensity = world.intensity_at(*light, input.point);
    EXPECT_EQ(intensity, input.result);
  }
  
}

TEST(WorldTest, IsShadowedFunctionMaterialDoesNotCastShadow) {

  auto w = world::World();
  w.light = light::PointLight(math::Point(-10, 5, 0), color::Color(1, 1, 1));
  auto plane = std::make_shared<geo::Plane>();
  plane->transform = math::rotation_z(M_PI / 2);
  w.objects.push_back(plane);
  
  auto point = math::Point(20, 0, 0);
  EXPECT_TRUE(w.is_shadowed(w.light->position, point));

  w.objects[0]->material.cast_shadow = false;
  EXPECT_FALSE(w.is_shadowed(w.light->position, point));
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
  auto i = geo::Intersection(1, shape.get());
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
  i = geo::Intersection(sqrt(2), shape2.get());
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
  auto i = geo::Intersection(sqrt(2), shape.get());
  auto comps = geo::prepare_computations(i, r);
  auto col = w.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.87676, 0.92434, 0.82917));
}

TEST(WorldTest, ReflectionAvoidInfiniteRecursion) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe"; // Otherwise memory leak

  // color_at with mutually reflective surfaces
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
  auto i = geo::Intersection(sqrt(2), shape.get());
  auto comps = geo::prepare_computations(i, r);
  auto col = w.reflected_color(comps, 0); //no recursive call remaining
  EXPECT_EQ(col, color::BLACK);
}

TEST(WorldTest, RefractionOpaqueObject) {

  // The refracted color with an opaque surface is black
  auto w = world::build_default_world();
  const auto& s = w.objects.at(0);
  auto r = ray::Ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto xs = geo::Intersections {geo::Intersection(4, s.get()), geo::Intersection(6, s.get())};
  auto comps = geo::prepare_computations(xs[0], r, xs);
  auto c = w.refracted_color(comps, 5);
  EXPECT_EQ(c, color::BLACK);
}

TEST(WorldTest, RefractionMaxRecursionLimit) {

  // If refracted_color is invoked with no more remaining calls, the color is black
  auto w = world::build_default_world();
  const auto& s = w.objects.at(0);
  s->material.transparency = 1;
  s->material.refractive_index = 1.5;
  auto r = ray::Ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto xs = geo::Intersections {geo::Intersection(4, s.get()), geo::Intersection(6, s.get())};
  auto comps = geo::prepare_computations(xs[0], r, xs);
  auto c = w.refracted_color(comps, 0);
  EXPECT_EQ(c, color::BLACK);
}

TEST(WorldTest, RefractionTotalInternalReflection) {

  // The refracted color under total internal reflection is black
  auto w = world::build_default_world();
  const auto& s = w.objects.at(0);
  s->material.transparency = 1;
  s->material.refractive_index = 1.5;
  auto r = ray::Ray(math::Point(0, 0, sqrt(2) / 2), math::Vector(0, 1, 0));
  auto xs = geo::Intersections {geo::Intersection(-sqrt(2)/2, s.get()), geo::Intersection(sqrt(2)/2, s.get())};
  // the ray originates from inside the sphere
  auto comps = geo::prepare_computations(xs[1], r, xs);
  auto c = w.refracted_color(comps);
  EXPECT_EQ(c, color::BLACK);
}

TEST(WorldTest, RefractionRefractedColor) {

  // The refracted color when the ray is refracted
  auto w = world::build_default_world();
  auto& obj1 = w.objects.at(0);
  obj1->material.ambient = 1;
  obj1->material.pattern = std::make_shared<pattern::TestPattern>();
  // interior sphere is glasssy
  auto& obj2 = w.objects.at(1);
  obj2->material.transparency = 1;
  obj2->material.refractive_index = 1.5;
  auto r = ray::Ray(math::Point(0, 0, 0.1), math::Vector(0, 1, 0));
  auto xs = geo::Intersections{geo::Intersection(-0.9899, obj1.get()),
			       geo::Intersection(-0.4899, obj2.get()),
			       geo::Intersection(0.4899, obj2.get()),
			       geo::Intersection(0.9899, obj1.get())};
  auto comps = geo::prepare_computations(xs[2], r, xs);
  auto c = w.refracted_color(comps);
  EXPECT_EQ(c, color::Color(0, 0.99888, 0.04722));
}

TEST(WorldTest, RefractionShadeHit) {

  // shade_hit with the default world, a semi transparent floor and a sphere below
  auto w = world::build_default_world();
  auto floor = std::make_shared<geo::Plane>();
  floor->transform = math::translation(0, -1, 0);
  floor->material.transparency = 0.5;
  floor->material.refractive_index = 1.5;
  w.objects.push_back(floor);

  auto ball = std::make_shared<geo::Sphere>();
  ball->material.color = color::Color(1, 0, 0);
  ball->material.ambient = 0.5;
  ball->transform = math::translation(0, -3.5, -0.5);
  w.objects.push_back(ball);
  
  auto r = ray::Ray(math::Point(0, 0, -3), math::Vector(0, -sqrt(2)/2, sqrt(2)/2));
  auto xs = geo::Intersections{geo::Intersection(sqrt(2), floor.get())};
  auto comps = geo::prepare_computations(xs[0], r, xs);
  auto col = w.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.93642, 0.68642, 0.68642));
}

TEST(WorldTest, FresnelEffectShadeHit) {

  // ShadeHit with a reflective transparent material
  auto w = world::build_default_world();
  auto r = ray::Ray(math::Point(0, 0, -3), math::Vector(0, -sqrt(2)/2, sqrt(2)/2));  
  auto floor = std::make_shared<geo::Plane>();
  floor->transform = math::translation(0, -1, 0);
  floor->material.reflective = 0.5;
  floor->material.transparency = 0.5;
  floor->material.refractive_index = 1.5;
  w.objects.push_back(floor);

  auto ball = std::make_shared<geo::Sphere>();
  ball->material.color = color::Color(1, 0, 0);
  ball->material.ambient = 0.5;
  ball->transform = math::translation(0, -3.5, -0.5);
  w.objects.push_back(ball);

  auto xs = geo::Intersections{geo::Intersection(sqrt(2), floor.get())};
  auto comps = geo::prepare_computations(xs[0], r, xs);
  auto col = w.shade_hit(comps);
  EXPECT_EQ(col, color::Color(0.93391, 0.69643, 0.69243));
}

