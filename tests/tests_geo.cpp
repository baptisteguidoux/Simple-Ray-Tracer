
#include <memory>

#include <gtest/gtest.h>

#include "ray.hpp"
#include "geo.hpp"


TEST(IntersectionTest, IntersectionDataStructure) {

  // For the moment, an Intersection will only contain two things:
  // the t value of the intersection, and the object that was intersected
  auto sphere = std::make_shared<geo::TestShape>();
  geo::Intersection inter(3.5, sphere);
  EXPECT_EQ(inter.t, 3.5);
  EXPECT_EQ(inter.geometry, sphere);

  // std::vector of Intersection(s)
  geo::Intersection inter1(1.0, sphere);
  geo::Intersection inter2(2.0, sphere);

  auto xss = geo::Intersections{inter1, inter2};
  EXPECT_EQ(xss.size(), 2);
  EXPECT_EQ(xss[0].t, 1);
  EXPECT_EQ(xss[1].t, 2);

  // Now there is an object variable member on Intersection
  EXPECT_EQ(xss[0].geometry, sphere);
  EXPECT_EQ(xss[1].geometry, sphere);

  // This test is in contradiction with the one that checks that a world, default construct, is built with two default spheres
  // auto sphere2 = geo::Sphere();
  // EXPECT_NE(xss[1].geometry, sphere2);
}

TEST(GeoTest, SphereIntesect){

  ray::Ray ray(math::Point(0, 0, -5.0), math::Vector(0, 0, 1));
  auto sph = std::make_shared<geo::Sphere>(); // suppose shere radius is one and it is positioned at (0, 0, 0)

  auto xs = sph->intersects(ray);
  // A ray intersets a Sphere at two points
  EXPECT_EQ(xs.size(), 2.0);
  // The ray intersects the Sphere at (0, 0, -1), 4 units away from ray origin
  EXPECT_EQ(xs[0].t, 4.0);
  // Then second intersection at (0, 0, 1)
  EXPECT_EQ(xs[1].t, 6.0);

  // Now the ray is moved up, so that it will intersects the Sphere at a tangent, intersecting just at one point
  ray::Ray ray2(math::Point(0.0, 1.0, -5.0), math::Vector(0.0, 0.0, 1.0));
  // Another Sphere
  auto sph2 = std::make_shared<geo::Sphere>();

  xs = sph2->intersects(ray2);
  // Even if the ray intersects only at one point
  // This will help when deternining if object overlaps
  EXPECT_EQ(xs.size(), 2);
  EXPECT_EQ(xs[0].t, 5);
  EXPECT_EQ(xs[1].t, 5);

  // Now the ray starts to high to intersect
  ray::Ray ray3(math::Point(0.0, 2.0, -5.0), math::Vector(0.0, 0.0, 1.0));
  auto sph3 = std::make_shared<geo::Sphere>();

  xs = sph3->intersects(ray3);
  EXPECT_EQ(xs.size(), 0);

  // What happens if the ray originates from inside the Sphere?
  // There is one intersection in front of the ray and also another one behind, the ray extends behind the origin!
  ray::Ray ray4(math::Point(0.0, 0.0, 0.0), math::Vector(0.0, 0.0, 1.0));
  auto sph4 = std::make_shared<geo::Sphere>();

  xs = sph4->intersects(ray4);
  EXPECT_EQ(xs.size(), 2);
  EXPECT_EQ(xs[0].t, -1);
  EXPECT_EQ(xs[1].t, 1);

  // If the Sphere is totally behind the ray's origin, we should still have 2 intersect points
  ray::Ray ray5(math::Point(0.0, 0.0, 5.0), math::Vector(0.0, 0.0, 1.0));
  auto sph5 = std::make_shared<geo::Sphere>();

  xs = sph5->intersects(ray5);
  EXPECT_EQ(xs.size(), 2);
  EXPECT_EQ(xs[0].t, -6);
  EXPECT_EQ(xs[1].t, -4);
}

