
#include <memory>
#include <cmath>
#include <algorithm>
#include <utility>

#include <gtest/gtest.h>

#include "ray.hpp"
#include "geo.hpp"
#include "tuple.hpp"
#include "matrix.hpp"
#include "material.hpp"


TEST(GeoTest, IntersectionDataStructure) {

  // For the moment, an Intersection will only contain two things:
  // the t value of the intersection, and the object that was intersected
  auto sphere = std::make_shared<geo::TestShape>();
  geo::Intersection inter(3.5, sphere.get());
  EXPECT_EQ(inter.t, 3.5);
  EXPECT_EQ(inter.geometry, sphere);

  // std::vector of Intersection(s)
  geo::Intersection inter1(1.0, sphere.get());
  geo::Intersection inter2(2.0, sphere.get());

  auto xss = geo::Intersections{inter1, inter2};
  EXPECT_EQ(xss.size(), 2);
  EXPECT_EQ(xss[0].t, 1);
  EXPECT_EQ(xss[1].t, 2);

  // Now there is an object variable member on Intersection
  EXPECT_EQ(xss[0].geometry, sphere);
  EXPECT_EQ(xss[1].geometry, sphere);
}

TEST(GeoTest, IntersectionCopyOperator) {

  auto shape = std::make_shared<geo::TestShape>();
  shape->transform = math::translation(0, 1, 0) * math::scaling(0.5, 1, 0.25);
  shape->material.pattern = std::make_shared<pattern::CheckerPattern>(color::BLACK, color::WHITE);

  geo::Intersection intersection1(1, shape.get());

  auto intersection2 = intersection1;

  EXPECT_EQ(*intersection1.geometry, *intersection2.geometry);
  EXPECT_EQ(intersection1.geometry->transform, intersection2.geometry->transform);
  EXPECT_EQ(intersection1.geometry->material, intersection2.geometry->material);
}

TEST(GeoTest, IntersectionCopyConstructor) {

  auto shape = std::make_shared<geo::TestShape>();
  shape->transform = math::translation(0, 1, 0) * math::scaling(0.5, 1, 0.25);
  shape->material.pattern = std::make_shared<pattern::CheckerPattern>(color::BLACK, color::WHITE);

  geo::Intersection intersection1(1, shape.get());
  geo::Intersection intersection2 {intersection1};

  EXPECT_EQ(*intersection1.geometry, *intersection2.geometry);
  EXPECT_EQ(intersection1.geometry->transform, intersection2.geometry->transform);
  EXPECT_EQ(intersection1.geometry->material, intersection2.geometry->material);
}

TEST(GeoTest, IntersectionsCopyOperator) {

  auto shape = std::make_shared<geo::TestShape>();
  shape->transform = math::translation(0, 1, 0) * math::scaling(0.5, 1, 0.25);
  shape->material.pattern = std::make_shared<pattern::CheckerPattern>(color::BLACK, color::WHITE);

  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::rotation_z(M_PI);
  sphere->material.color = color::Color(1, 0.2, 0.45);

  auto plane = std::make_shared<geo::Plane>();
  plane->transform = math::rotation_z(M_PI / 2) * math::scaling(0.25, 0.25, 0.25);
  plane->material.specular = 0.2;

  geo::Intersections intersections1 {
    geo::Intersection(0, shape.get()),
    geo::Intersection(1, sphere.get()),
    geo::Intersection(2, plane.get()),
  };

  auto intersections2 = intersections1;

  for (size_t i = 0; i < intersections1.size(); i++) {
    EXPECT_EQ(intersections1[i], intersections2[i]);
    EXPECT_EQ(*intersections1[i].geometry, *intersections2[i].geometry);
    EXPECT_EQ(intersections1[i].geometry->transform, intersections2[i].geometry->transform);
    EXPECT_EQ(intersections1[i].geometry->material, intersections2[i].geometry->material);
  }

}

TEST(GeoTest, IntersectionsCopyConstructor) {

  auto shape = std::make_shared<geo::TestShape>();
  shape->transform = math::translation(0, 1, 0) * math::scaling(0.5, 1, 0.25);
  shape->material.pattern = std::make_shared<pattern::CheckerPattern>(color::BLACK, color::WHITE);

  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::rotation_z(M_PI);
  sphere->material.color = color::Color(1, 0.2, 0.45);

  auto plane = std::make_shared<geo::Plane>();
  plane->transform = math::rotation_z(M_PI / 2) * math::scaling(0.25, 0.25, 0.25);
  plane->material.specular = 0.2;

  geo::Intersections intersections1 {
    geo::Intersection(0, shape.get()),
    geo::Intersection(1, sphere.get()),
    geo::Intersection(2, plane.get()),
  };

  geo::Intersections intersections2 {intersections1};

  for (size_t i = 0; i < intersections1.size(); i++) {
    EXPECT_EQ(intersections1[i], intersections2[i]);
    EXPECT_EQ(*intersections1[i].geometry, *intersections2[i].geometry);
    EXPECT_EQ(intersections1[i].geometry->transform, intersections2[i].geometry->transform);
    EXPECT_EQ(intersections1[i].geometry->material, intersections2[i].geometry->material);
  }

}

TEST(GeoTest, IntersectionMayHaveUAndVProperties) {

  auto triangle = std::make_shared<geo::Triangle>(math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  auto ixs_with_uv = geo::Intersection(3.5, triangle.get(), 0.2, 0.4);
  EXPECT_TRUE(math::almost_equal(ixs_with_uv.u, 0.2));
  EXPECT_TRUE(math::almost_equal(ixs_with_uv.v, 0.4));
}

TEST(GeoTest, SphereEquality) {

  auto sphere1 = std::make_shared<geo::Sphere>();
  auto sphere2 = std::make_shared<geo::Sphere>();
  auto sphere3 = std::make_shared<geo::Sphere>();

  sphere1->transform = math::translation(1, 2, 3)* math::scaling(2, 2, 2);
  sphere1->material.color = color::Color(1, 0.2, 0.3);
  sphere3->transform = math::translation(1, 2, 3)* math::scaling(2, 2, 2);
  sphere3->material.color = color::Color(1, 0.2, 0.3);

  EXPECT_NE(*sphere1, *sphere2);
  EXPECT_EQ(*sphere1, *sphere3);
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

TEST(GeoTest, Hits) {

  // The hit when all intersections have a positive t
  auto s = std::make_shared<geo::Sphere>();
  auto inter1 = geo::Intersection(1.0, s.get());
  auto inter2 = geo::Intersection(2.0, s.get());
  auto xss = geo::Intersections {inter1, inter2};

  auto h = geo::hit(xss);
  EXPECT_EQ(h, inter1);

  // The hit, when some intersections have a negative t
  auto inter3 = geo::Intersection(-1.0, s.get());
  auto inter4 = geo::Intersection(2.0, s.get());
  xss = geo::Intersections {inter3, inter4};

  auto h2 = geo::hit(xss);
  EXPECT_EQ(h2, inter4);

  // The hit when all intersections have a negative t
  auto inter5 = geo::Intersection(-2.0, s.get());
  auto inter6 = geo::Intersection(-1.0, s.get());
  xss = geo::Intersections {inter5, inter6};

  auto h3 = geo::hit(xss);
  EXPECT_EQ(h3, std::nullopt); // std::optional void result

  // The hit is always the lowest nonegative intersection
  auto inter7 = geo::Intersection(5.0, s.get());
  auto inter8 = geo::Intersection(7.0, s.get());
  auto inter9 = geo::Intersection(-3.0, s.get());
  auto inter10 = geo::Intersection(2.0, s.get());
  xss = geo::Intersections {inter7, inter8, inter9, inter10};

  EXPECT_EQ(geo::hit(xss), inter10);
}

TEST(GeoTest, ShapeTransforms) {

  // A shape has a default transform
  auto test_shape = std::make_shared<geo::TestShape>();
  EXPECT_EQ(test_shape->transform, math::IDENTITY_MATRIX);

  // Assigning a transform
  test_shape->transform = math::translation(2, 3, 4);
  EXPECT_EQ(test_shape->transform, math::translation(2, 3, 4));

  // TestShape is derived from Shape
  EXPECT_NE(dynamic_cast<geo::Shape*>(test_shape.get()), nullptr);
}

TEST(GeoTest, IntersectTestShapeRay) {

  // Intersecting a scaled shape with a ray
  auto r = ray::Ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto s = std::make_shared<geo::TestShape>();
  s->transform = math::scaling(2, 2, 2);
  auto ixs = s->intersects(r);
  // intersects calls local_intersects
  EXPECT_EQ(s->saved_ray.origin, math::Point(0, 0, -2.5));
  EXPECT_EQ(s->saved_ray.direction, math::Vector(0, 0, 0.5));

  // Intersecting a translated shape with a ray
  r = ray::Ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  s = std::make_shared<geo::TestShape>();
  s->transform = math::translation(5, 0, 0);
  ixs = s->intersects(r);
  EXPECT_EQ(s->saved_ray.origin, math::Point(-5, 0, -5));
  EXPECT_EQ(s->saved_ray.direction, math::Vector(0, 0, 1));
}

TEST(GeoTest, CalculateNormalOnSphere) {

  // Assume the given point is always on the sphere
  auto sphere = std::make_shared<geo::Sphere>();

  // Normal on a sphere at a point on the x axis
  auto normal1 = sphere->normal_at(math::Point(1.0, 0.0, 0.0), geo::Intersection(INFINITY, sphere.get()));
  EXPECT_EQ(normal1, math::Vector(1.0, 0.0, 0.0));

  // Normal on a sphere at a point on the y axis
  auto normal2 = sphere->normal_at(math::Point(0.0, 1.0, 0.0), geo::Intersection(INFINITY, sphere.get()));
  EXPECT_EQ(normal2, math::Vector(0.0, 1.0, 0.0));

  // Normal on a sphere at a point on the z axis
  auto normal3 = sphere->normal_at(math::Point(0.0, 0.0, 1.0), geo::Intersection(INFINITY, sphere.get()));
  EXPECT_EQ(normal3, math::Vector(0.0, 0.0, 1.0));

  // The normal on a sphere at a nonaxial point
  auto normal4 = sphere->normal_at(math::Point(sqrt(3) / 3, sqrt(3) / 3, sqrt(3) / 3), geo::Intersection(INFINITY, sphere.get()));
  EXPECT_EQ(normal4, math::Vector(sqrt(3) / 3, sqrt(3) / 3, sqrt(3) / 3));

  // A surface normal should always be normalized
  EXPECT_EQ(normal4, math::normalize(normal4));

  // Computing the normal on a translated shape
  auto shape = std::make_shared<geo::TestShape>();
  shape->transform = math::translation(0, 1, 0);
  auto normal7 = shape->normal_at(math::Point(0, 1.70711, -0.70711), geo::Intersection(INFINITY, shape.get()));
  EXPECT_EQ(normal7, math::Vector(0, 0.70711, -0.70711));

  // Computing the normal on a transformed shape
  shape = std::make_shared<geo::TestShape>();
  shape->transform = math::scaling(1, 0.5, 1) * math::rotation_z(M_PI / 5);
  auto normal8 = shape->normal_at(math::Point(0, sqrt(2) / 2, -sqrt(2) / 2), geo::Intersection(INFINITY, shape.get()));
  EXPECT_EQ(normal8, math::Vector(0, 0.97014, -0.24254));
}

TEST(GeoTest, NormalOnPlane) {

  // The normal of a plane is constant everywhere
  auto plane = std::make_shared<geo::Plane>();

  // Check it is subclass of Shape
  EXPECT_NE(dynamic_cast<geo::Shape*>(plane.get()), nullptr);
  
  auto norm1 = plane->local_normal_at(math::Point(0, 0, 0), geo::Intersection(INFINITY, plane.get()));
  auto norm2 = plane->local_normal_at(math::Point(10, 0, -10), geo::Intersection(INFINITY, plane.get()));
  auto norm3 = plane->local_normal_at(math::Point(-5, 0, 150), geo::Intersection(INFINITY, plane.get()));

  EXPECT_EQ(norm1, math::Vector(0, 1, 0));
  EXPECT_EQ(norm2, math::Vector(0, 1, 0));
  EXPECT_EQ(norm3, math::Vector(0, 1, 0));
}

TEST(GeoTest, IntersectsPlane) {

  // Intersects with a ray parallel to the Plane
  auto plane = std::make_shared<geo::Plane>();
  auto ry = ray::Ray(math::Point(0, 10, 0), math::Vector(0, 0, 1));
  auto ixs = plane->local_intersects(ry);
  EXPECT_EQ(ixs.size(), 0);

  // Intersects with a ray coplanar to the Plane
  plane = std::make_shared<geo::Plane>();
  ry = ray::Ray(math::Point(0, 0, 0), math::Vector(0, 0, 1));
  ixs = plane->local_intersects(ry);
  EXPECT_EQ(ixs.size(), 0);

  // A ray intersecting a Plane from above
  plane = std::make_shared<geo::Plane>();
  ry = ray::Ray(math::Point(0, 1, 0), math::Vector(0, -1, 0));
  ixs = plane->local_intersects(ry);
  EXPECT_EQ(ixs.size(), 1);
  EXPECT_EQ(ixs[0].t, 1);
  EXPECT_EQ(*(ixs[0].geometry), *plane);

  // A ray intersecting a Plane from below
  plane = std::make_shared<geo::Plane>();
  ry = ray::Ray(math::Point(0, -1, 0), math::Vector(0, 1, 0));
  ixs = plane->local_intersects(ry);
  EXPECT_EQ(ixs.size(), 1);
  EXPECT_EQ(ixs[0].t, 1);
  EXPECT_EQ(*(ixs[0].geometry), *plane);
}

TEST(GeoTest, RayIntersectsCube) {

  // Rays intersect with Cube
  struct TestInput {
    math::Point origin;
    math::Vector direction;
    int t1;
    int t2;

    TestInput(const math::Point& o, const math::Vector& d, const int t1_, const int t2_) :
      origin {o}, direction {d}, t1 {t1_}, t2 {t2_} {}
  };

  std::vector<TestInput> inputs{
    TestInput(math::Point(5, 0.5, 0), math::Vector(-1, 0, 0), 4, 6), // +x
    TestInput(math::Point(-5, 0.5, 0), math::Vector(1, 0, 0), 4, 6), // -x
    TestInput(math::Point(0.5, 5, 0), math::Vector(0, -1, 0), 4, 6), // +y
    TestInput(math::Point(0.5, -5, 0), math::Vector(0, 1, 0), 4, 6), // -y
    TestInput(math::Point(0.5, 0, 5), math::Vector(0, 0, -1), 4, 6), // +z
    TestInput(math::Point(0.5, 0, -5), math::Vector(0, 0, 1), 4, 6), // -z
    TestInput(math::Point(0, 0.5, 0), math::Vector(0, 0, 1), -1, 1), // ray from inside the cube
  };

  auto cube = std::make_shared<geo::Cube>();

  for (const auto& input: inputs) {
    auto r = ray::Ray(input.origin, input.direction);
    auto xs = cube->local_intersects(r);
    EXPECT_EQ(xs.size(), 2);
    EXPECT_EQ(xs.at(0).t, input.t1);
    EXPECT_EQ(xs.at(1).t, input.t2);
  }
  
}

TEST(GeoTest, RayDoesNotIntersectsCube) {

  struct TestInput {
    math::Point origin;
    math::Vector direction;

    TestInput(const math::Point& o, const math::Vector& d) : origin {o}, direction {d} {}
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(-2, 0, 0), math::Vector(0.2673, 0.5345, 0.8018)),
    TestInput(math::Point(0, -2, 0), math::Vector(0.8018, 0.2673, 0.5345)),
    TestInput(math::Point(0, 0, -2), math::Vector(0.5345, 0.8018, 0.2673)),
    TestInput(math::Point(2, 0, 2), math::Vector(0, 0, -1)),
    TestInput(math::Point(0, 2, 2), math::Vector(0, -1, 0)),
    TestInput(math::Point(2, 2, 0), math::Vector(-1, 0, 0)),
  };
 
  // Rays do not intersect with Cube
  auto c = std::make_shared<geo::Cube>();

  for (const auto& input : inputs) {
    const auto r = ray::Ray(input.origin, input.direction);
    const auto xs = c->local_intersects(r);
    EXPECT_EQ(xs.size(), 0);
  }
  
}

TEST(GeoTest, NormalOnCube) {

  struct TestInput {
    math::Point point;
    math::Vector normal;

    TestInput(const math::Point& p, const math::Vector& n) : point {p}, normal {n} {}
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(1, 0.5, -0.8), math::Vector(1, 0, 0)),
    TestInput(math::Point(-1, -0.2, 0.9), math::Vector(-1, 0, 0)),
    TestInput(math::Point(-0.4, 1, -0.1), math::Vector(0, 1, 0)),
    TestInput(math::Point(0.3, -1, -0.7), math::Vector(0, -1, 0)),
    TestInput(math::Point(-0.6, 0.3, 1), math::Vector(0, 0, 1)),
    TestInput(math::Point(0.4, 0.4, -1), math::Vector(0, 0, -1)),
    TestInput(math::Point(1, 1, 1), math::Vector(1, 0, 0)),
    TestInput(math::Point(-1, -1, -1), math::Vector(-1, 0, 0)),
      
  };  
  // The normal on the surface of a cube
  auto c = std::make_shared<geo::Cube>();
  
  for (const auto& input : inputs){
    auto p = input.point;
    auto normal = c->local_normal_at(p, geo::Intersection(INFINITY, c.get()));
    EXPECT_EQ(normal, input.normal);
  }
 
}

TEST(GeoTest, CylinderEquality) {

  auto cyl1 = std::make_shared<geo::Cylinder>();
  auto cyl2 = std::make_shared<geo::Cylinder>();
  auto cyl3 = std::make_shared<geo::Cylinder>();

  cyl1->transform = math::scaling(1, 2, 3);
  cyl2->transform = math::scaling(1, 2, 3);
  cyl3->transform = math::scaling(1, 2, 3);

  cyl1->minimum = 0;
  cyl1->maximum = 2;
  cyl1->closed = true;

  cyl3->minimum = 0;
  cyl3->maximum = 2;
  cyl3->closed = true;

  EXPECT_NE(*cyl1, *cyl2);
  EXPECT_EQ(*cyl1, *cyl3);
}

TEST(GeoTest, CylinderRayMisses) {

  // A ray misses a cylinder
  
  struct TestInput {
    math::Point origin;
    math::Vector direction;

    TestInput(const math::Point& o, const math::Vector& d) : origin {o}, direction {d} {}
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(1, 0, 0), math::Vector(0, 1, 0)),
    TestInput(math::Point(0, 0, 0), math::Vector(0, 1, 0)),
    TestInput(math::Point(0, 0, -5), math::Vector(1, 1, 1)),
      };

  auto cyl = std::make_shared<geo::Cylinder>();
  for (const auto& input : inputs) {
    auto dir = math::normalize(input.direction);
    auto r = ray::Ray(input.origin, dir);
    auto xs = cyl->local_intersects(r);
    EXPECT_EQ(xs.size(), 0);
  }

}

TEST(GeoTest, CylinderRayHits) {

  // A ray hits a cylinder

    struct TestInput {
    math::Point origin;
    math::Vector direction;
    double t1;
    double t2;

      TestInput(const math::Point& o, const math::Vector& d, const double t1_, const double  t2_) :
	origin {o}, direction {d}, t1 {t1_}, t2 {t2_ } {}
  };

  std::vector<TestInput> inputs {
    // hits the tangent
    TestInput(math::Point(1, 0, -5), math::Vector(0, 0, 1),  5, 5),
    // hits perpendiculary
    TestInput(math::Point(0, 0, -5), math::Vector(0, 0, 1), 4, 6),
    // strikes at an angles
    TestInput(math::Point(0.5, 0, -5), math::Vector(0.1, 1, 1), 6.80798, 7.08872),
  };

  auto cyl = std::make_shared<geo::Cylinder>();
  for (const auto& input : inputs) {
    auto dir = math::normalize(input.direction);
    auto r = ray::Ray(input.origin, dir);
    auto xs = cyl->local_intersects(r);
    EXPECT_EQ(xs.size(), 2);
    EXPECT_TRUE(math::almost_equal(xs[0].t, input.t1));
    EXPECT_TRUE(math::almost_equal(xs[1].t, input.t2));
  }
}

TEST(GeoTest, CylinderNormalVector) {

  // Normal vector on a cylinder
  auto cyl = std::make_shared<geo::Cylinder>();
  
  EXPECT_EQ(cyl->local_normal_at(math::Point(1, 0, 0), geo::Intersection(INFINITY, cyl.get())), math::Vector(1, 0, 0));
  EXPECT_EQ(cyl->local_normal_at(math::Point(0, 5, -1), geo::Intersection(INFINITY, cyl.get())), math::Vector(0, 0, -1));
  EXPECT_EQ(cyl->local_normal_at(math::Point(0, -2, 1), geo::Intersection(INFINITY, cyl.get())), math::Vector(0, 0, 1));
  EXPECT_EQ(cyl->local_normal_at(math::Point(-1, -1, 0), geo::Intersection(INFINITY, cyl.get())), math::Vector(-1, 0, 0));
}

TEST(GeoTest, CylinderMaximumMinimumBounds) {

  // The default minimum and maximum for a cylinder
  auto cyl = geo::Cylinder();
  EXPECT_EQ(cyl.minimum, -INFINITY);
  EXPECT_EQ(cyl.maximum, INFINITY);
}

TEST(GeoTest, CylinderIntersectingTruncatedCylinder) {

  // Intersecting a constrained cylinder
  auto cyl = std::make_shared<geo::Cylinder>();
  cyl->minimum = 1;
  cyl->maximum = 2;

  struct TestInput {
    math::Point origin;
    math::Vector direction;
    int count;

    TestInput(const math::Point& o, const math::Vector& d, const int c) :
        origin {o}, direction {d}, count {c} {}
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(0, 1.5, 0), math::Vector(0.1, 1, 0), 0),
    TestInput(math::Point(0, 3, -5), math::Vector(0, 0, 1), 0),
    TestInput(math::Point(0, 0, -5), math::Vector(0, 0, 1), 0),
    TestInput(math::Point(0, 2, -5), math::Vector(0, 0, 1), 0),
    TestInput(math::Point(0, 1, -5), math::Vector(0, 0, 1), 0),
    TestInput(math::Point(0, 1.5, -2), math::Vector(0, 0, 1), 2),
  };

  for (const auto& input : inputs) {
    auto direction = math::normalize(input.direction);
    auto r = ray::Ray(input.origin, direction);
    auto xs = cyl->local_intersects(r);
    EXPECT_EQ(xs.size(), input.count);
  }

}

TEST(GeoTest, CylinderCloseAttribute) {

  // A cylinder has a closed attribute that defaults to false
  auto cyl = geo::Cylinder();
  EXPECT_FALSE(cyl.closed);
}

TEST(GeoTest, ClosedCylinderIntersection) {

  // Intersects the caps of a closed cylinder
  auto cyl = std::make_shared<geo::Cylinder>();
  cyl->minimum = 1;
  cyl->maximum = 2;
  cyl->closed = true;

  struct TestInput {
    math::Tuple point;
    math::Tuple direction;
    int count;

    TestInput(const math::Tuple& p, const math::Tuple& d, const int c) : point {p}, direction {d}, count {c} {}
  };

  std::vector<TestInput> test_inputs {
    TestInput{math::Point(0, 3, 0), math::Vector(0, -1, 0), 2},
    TestInput{math::Point(0, 3, -2), math::Vector(0, -1, 2), 2},
    TestInput{math::Point(0, 4, -2), math::Vector(0, -1, 1), 2},
    TestInput{math::Point(0, 0, -2), math::Vector(0, 1, 2), 2},
    TestInput{math::Point(0, -1, -2), math::Vector(0, 1, 1), 2},
  };

  for (const auto& input : test_inputs) {
    auto dir = math::normalize(input.direction);
    auto r = ray::Ray(input.point, dir);
    auto xs = cyl->local_intersects(r);
    EXPECT_EQ(xs.size(), input.count);
  }
}

TEST(GeoTest, ClosedCylinderNormalAt) {

  // Find the normal vector on a closed cylinder's end caps
  auto cyl = std::make_shared<geo::Cylinder>();
  cyl->minimum = 1;
  cyl->maximum = 2;
  cyl->closed = true;

  struct TestInput {
    math::Tuple point;
    math::Tuple normal;

    TestInput(const math::Tuple& p, const math::Tuple& n) : point {p}, normal {n} {}
  };

  std::vector<TestInput> test_inputs {
    TestInput(math::Point(0, 1, 0), math::Vector(0, -1, 0)),
    TestInput(math::Point(0.5, 1, 0), math::Vector(0, -1, 0)),
    TestInput(math::Point(0, 1, 0.5), math::Vector(0, -1, 0)),
    TestInput(math::Point(0, 2, 0), math::Vector(0, 1, 0)),
    TestInput(math::Point(0.5, 2, 0), math::Vector(0, 1, 0)),
    TestInput(math::Point(0, 2, 0.5), math::Vector(0, 1, 0)),
  };

  for (const auto& input : test_inputs) {
    auto vec = cyl->local_normal_at(input.point, geo::Intersection(INFINITY, cyl.get()));
    EXPECT_EQ(input.normal, vec);
  }
  
}

TEST(GeoTest, DoubleConeDefault) {

  // The default minimum and maximum for a cylinder
  auto cone = geo::DoubleCone();
  EXPECT_EQ(cone.minimum, -INFINITY);
  EXPECT_EQ(cone.maximum, INFINITY);
  EXPECT_FALSE(cone.closed);
}

TEST(GeoTest, DoubleConeEquality) {

  auto con1 = std::make_shared<geo::DoubleCone>();
  auto con2 = std::make_shared<geo::DoubleCone>();
  auto con3 = std::make_shared<geo::DoubleCone>();

  con1->transform = math::scaling(1, 2, 3);
  con2->transform = math::scaling(1, 2, 3);
  con3->transform = math::scaling(1, 2, 3);

  con1->minimum = 0;
  con1->maximum = 2;
  con1->closed = true;

  con3->minimum = 0;
  con3->maximum = 2;
  con3->closed = true;

  EXPECT_NE(*con1, *con2);
  EXPECT_EQ(*con1, *con3);
}

TEST(GeoTest, DoubleConeIntersects) {

  // Intersecting a double cone with a ray
  auto cone = std::make_shared<geo::DoubleCone>();

  struct TestInput {
    math::Tuple origin;
    math::Tuple direction;
    float t0;
    float t1;

    TestInput(const math::Tuple& o, const math::Tuple& d, const float t0_, const float t1_) : origin {o}, direction {d}, t0 {t0_}, t1 {t1_} {}
  };

  std::vector<TestInput> test_inputs {
    TestInput(math::Point(0, 0, -5), math::Vector(0, 0, 1), 5, 5),
    TestInput(math::Point(0, 0, -5), math::Vector(1, 1, 1), 8.66025, 8.66025),
    TestInput(math::Point(1, 1, -5), math::Vector(-0.5, -1, 1), 4.55006, 49.44994),
  };

  for (const auto& input : test_inputs) {
    auto direction = math::normalize(input.direction);
    auto r = ray::Ray(input.origin, direction);
    auto xs = cone->local_intersects(r);
    ASSERT_EQ(xs.size(), 2);
    EXPECT_TRUE(math::almost_equal(xs[0].t, input.t0));
    EXPECT_TRUE(math::almost_equal(xs[1].t, input.t1));
  }

}

TEST(GeoTest, DoubleConeIntersectsWithARayParallelToOneHalve) {

  // Intersecting a double cone with a ray parallel to one of cones' halves
  auto cone = std::make_shared<geo::DoubleCone>();
  auto direction = math::normalize(math::Vector(0, 1, 1));
  auto r = ray::Ray(math::Point(0, 0, -1), direction);
  auto xs = cone->local_intersects(r);
  ASSERT_EQ(xs.size(), 1);
  EXPECT_TRUE(math::almost_equal(xs[0].t, 0.35355));
}

TEST(GeoTest, DoubleConeEndCapsIntersections) {

  // Intersecting a cone's end caps
  auto cone = std::make_shared<geo::DoubleCone>();

  cone->minimum = -0.5;
  cone->maximum = 0.5;
  cone->closed = true;

  struct TestInput {
    math::Tuple origin;
    math::Tuple direction;
    int count;

    TestInput(const math::Tuple& o, const math::Tuple& d, const int c) : origin {o}, direction {d}, count {c} {}
  };

  std::vector<TestInput> test_inputs {
    TestInput(math::Point(0, 0, -5), math::Vector(0, 1,  0), 0),
    TestInput(math::Point(0, 0, -0.25), math::Vector(0, 1, 1), 2),
    TestInput(math::Point(0, 0, -0.25), math::Vector(0, 1, 0), 4),      
  };

  for (const auto& input : test_inputs) {
    auto direction = math::normalize(input.direction);
    auto r = ray::Ray(input.origin, direction);
    auto xs = cone->local_intersects(r);
    EXPECT_EQ(xs.size(), input.count);
  }
  
}

TEST(GeoTest, DoubleConeNormalVector) {

  // Computing the normal vector on a DoubleCone
  auto cone = std::make_shared<geo::DoubleCone>();

  struct TestInput {

    math::Tuple point;
    math::Tuple normal;

    TestInput(const math::Tuple& p, const math::Tuple& n) : point {p}, normal {n} {}
  };

  std::vector<TestInput> test_inputs {
    TestInput(math::Point(0, 0, 0), math::Vector(0, 0, 0)),
    TestInput(math::Point(1, 1, 1), math::Vector(1, -sqrt(2), 1)),
    TestInput(math::Point(-1, -1, 0), math::Vector(-1, 1, 0)),
  };

  for (const auto& input : test_inputs) {
    auto norm = cone->local_normal_at(input.point, geo::Intersection(INFINITY, cone.get()));
    EXPECT_EQ(norm, input.normal);
  }

}

TEST(GeoTest, TriangleConstructor) {

  // The triangle constructor should precompute the two edge vectors and triangle normal
  math::Point p1(0, 1, 0);
  math::Point p2(-1, 0, 0);
  math::Point p3(1, 0, 0);

  auto triangle = std::make_shared<geo::Triangle>(p1, p2, p3);

  EXPECT_EQ(triangle->p1, p1);
  EXPECT_EQ(triangle->p2, p2);
  EXPECT_EQ(triangle->p3, p3);
  EXPECT_EQ(triangle->e1, math::Vector(-1, -1, 0));
  EXPECT_EQ(triangle->e2, math::Vector(1, -1, 0));
  EXPECT_EQ(triangle->normal, math::Vector(0, 0, -1));
}

TEST(GeoTest, TriangleNormalVector) {

  // The triangle's precomputed normal is used for every point of the triangle
  auto triangle = std::make_shared<geo::Triangle>(math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  auto n1 = triangle->local_normal_at(math::Point(0, 0.5, 0), geo::Intersection(INFINITY, triangle.get()));
  auto n2 = triangle->local_normal_at(math::Point(-0.5, 0.75, 0), geo::Intersection(INFINITY, triangle.get()));
  auto n3 = triangle->local_normal_at(math::Point(0.5, 0.25, 0), geo::Intersection(INFINITY, triangle.get()));
  EXPECT_EQ(n1, triangle->normal);
  EXPECT_EQ(n2, triangle->normal);
  EXPECT_EQ(n3, triangle->normal);  
}

TEST(GeoTest, TriangleEquality) {

  auto triangle1 = std::make_shared<geo::Triangle>(math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  auto triangle2 = std::make_shared<geo::Triangle>(math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  auto triangle3 = std::make_shared<geo::Triangle>(math::Point(1, 0, 0), math::Point(1, 1, 0), math::Point(1, 0, 0));

  EXPECT_EQ(*triangle1, *triangle2);
  EXPECT_NE(*triangle1, *triangle3);
}

// A Ray that misses a Triangle should not add any Intersection
// A Ray that strikes a Triangle should add one Intersection
TEST(GeoTest, TriangleIntersectsParallelRay) {

  auto triangle = std::make_shared<geo::Triangle>
    (math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  ray::Ray r(math::Point(0, -1, -2), math::Vector(0, 1, 0));
  auto xs = triangle->local_intersects(r);
  EXPECT_EQ(xs.size(), 0);
}

TEST(GeoTest, TriangleIntersectsRayPassBeyondP1P3Edge) {

  // The Ray pass beyond the Triangle p1-p3 edge
  auto triangle = std::make_shared<geo::Triangle>
    (math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  ray::Ray r(math::Point(1, 1, -2), math::Vector(0, 0, 1));
  auto xs = triangle->local_intersects(r);
  EXPECT_EQ(xs.size(), 0);
}

TEST(GeoTest, TriangleIntersectsRayPassBeyondP1P2Edge) {

  auto triangle = std::make_shared<geo::Triangle>
    (math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  ray::Ray r(math::Point(-1, 1, -2), math::Vector(0, 0, 1));
  auto xs = triangle->local_intersects(r);
  EXPECT_EQ(xs.size(), 0);
}

TEST(GeoTest, TriangleIntersectsRayPassBeyondP2P3Edge) {

  auto triangle = std::make_shared<geo::Triangle>
    (math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  ray::Ray r(math::Point(0, -1, -2), math::Vector(0, 0, 1));
  auto xs = triangle->local_intersects(r);
  EXPECT_EQ(xs.size(), 0);
}

TEST(GeoTest, TriangleIntersectsRayStrikes) {

  auto triangle = std::make_shared<geo::Triangle>
    (math::Point(0, 1, 0), math::Point(-1, 0, 0), math::Point(1, 0, 0));
  ray::Ray r(math::Point(0, 0.5, -2), math::Vector(0, 0, 1));
  auto xs = triangle->local_intersects(r);
  ASSERT_EQ(xs.size(), 1);
  EXPECT_EQ(xs[0].t, 2);
}

TEST(GeoTest, SmoothTriangleConstructor) {

  math::Point p1(0, 1, 0);
  math::Point p2(-1, 0, 0);
  math::Point p3(1, 0, 0);
  math::Vector n1(0, 1, 0);
  math::Vector n2(-1, 0, 0);
  math::Vector n3(1, 0, 0);    
  auto smoothtri = std::make_shared<geo::SmoothTriangle>(p1, p2, p3, n1, n2, n3);

  EXPECT_EQ(smoothtri->p1, p1);
  EXPECT_EQ(smoothtri->p2, p2);
  EXPECT_EQ(smoothtri->p3, p3);
  EXPECT_EQ(smoothtri->n1, n1);
  EXPECT_EQ(smoothtri->n2, n2);
  EXPECT_EQ(smoothtri->n3, n3);
}

TEST(GeoTest, SmoothTriangleIntersectionStoresUV) {

  math::Point p1(0, 1, 0);
  math::Point p2(-1, 0, 0);
  math::Point p3(1, 0, 0);
  math::Vector n1(0, 1, 0);
  math::Vector n2(-1, 0, 0);
  math::Vector n3(1, 0, 0);    
  auto smoothtri = std::make_shared<geo::SmoothTriangle>(p1, p2, p3, n1, n2, n3);
  ray::Ray r(math::Point(-0.2, 0.3, -2), math::Vector(0, 0, 1));
  auto xs = smoothtri->local_intersects(r);
  ASSERT_EQ(xs.size(), 1);
  EXPECT_TRUE(math::almost_equal(xs[0].u, 0.45));
  EXPECT_TRUE(math::almost_equal(xs[0].v, 0.25));
}

TEST(GeoTest, SmoothTriangleInterpolateNormalUsingUV) {

  math::Point p1(0, 1, 0);
  math::Point p2(-1, 0, 0);
  math::Point p3(1, 0, 0);
  math::Vector n1(0, 1, 0);
  math::Vector n2(-1, 0, 0);
  math::Vector n3(1, 0, 0);    
  auto smoothtri = std::make_shared<geo::SmoothTriangle>(p1, p2, p3, n1, n2, n3);

  geo::Intersection ix(1, smoothtri.get(), 0.45, 0.25);
  auto n = smoothtri->normal_at(math::Point(0, 0, 0), ix);
  EXPECT_EQ(n, math::Vector(-0.5547, 0.83205, 0));
}

TEST(GeoTest, SmoothTrianglePrepareComputationsOnNormal) {

  math::Point p1(0, 1, 0);
  math::Point p2(-1, 0, 0);
  math::Point p3(1, 0, 0);
  math::Vector n1(0, 1, 0);
  math::Vector n2(-1, 0, 0);
  math::Vector n3(1, 0, 0);    
  auto smoothtri = std::make_shared<geo::SmoothTriangle>(p1, p2, p3, n1, n2, n3);

  geo::Intersection ix(1, smoothtri.get(), 0.45, 0.25);
  ray::Ray r(math::Point(-0.2, 0.3, -2), math::Vector(0, 0, 1));
  geo::Intersections ixs {ix};
  auto comps = geo::prepare_computations(ix, r, ixs);

  EXPECT_EQ(comps.normal_vector, math::Vector(-0.5547, 0.83205, 0));
}

TEST(GeoTest, GroupShape) {

  // Creating a new group
  auto group = geo::Group();
  
  EXPECT_EQ(group.transform, math::IDENTITY_MATRIX);
  EXPECT_EQ(group.shapes.size(), 0);
}

TEST(GeoTest, GroupEquality) {

  // Checks Group Equality
  auto group1 = std::make_shared<geo::Group>();
  auto group2 = std::make_shared<geo::Group>();
  auto group3 = std::make_shared<geo::Group>();

  group1->transform = math::rotation_z(M_PI / 2);
  group2->transform = math::rotation_z(M_PI / 2);
  group3->transform = math::rotation_z(M_PI / 2);

  group1->material.specular = 0.2;
  group2->material.specular = 0.2;
  group3->material.specular = 0.2;

  auto sphere1 = std::make_shared<geo::Sphere>();
  auto sphere2 = std::make_shared<geo::Sphere>();
  sphere2->transform = math::scaling(1, 0.5, 0.25);

  group1->add_child(sphere1);
  group2->add_child(sphere2);
  group3->add_child(sphere1);

  EXPECT_NE(*group1, *group2);
  EXPECT_EQ(*group1, *group3);
}

TEST(GeoTest, GroupWithSubGroupsEquality) {

  // Checks Group Equality, when a Group contains another Group
  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::scaling(1, 2, 3);

  auto group1 = std::make_shared<geo::Group>();
  auto group2 = std::make_shared<geo::Group>();
  
  group1->add_child(group2);
  group2->add_child(sphere);

  auto group3 = std::make_shared<geo::Group>();
  auto group4 = std::make_shared<geo::Group>();
  
  group3->add_child(group4);
  group4->add_child(sphere);

  EXPECT_EQ(*group1, *group3);

  group4->transform = math::translation(0, 1, 2);

  EXPECT_NE(*group1, *group3);
}

TEST(GeoTest, ShapeHasParentAttribute) {

  // A Shape has a parent variable
  auto shape = std::make_shared<geo::TestShape>();
  
  EXPECT_EQ(shape->parent.lock(), nullptr);
}

TEST(GeoTest, GroupShapeAddChild) {

  // Adding a child to a Group
  auto group = std::make_shared<geo::Group>();
  auto shape = std::make_shared<geo::TestShape>();
  group->add_child(shape);
  
  EXPECT_NE(group->shapes.size(), 0);

  // Test group includes shape
  auto it = std::find_if(group->shapes.begin(), group->shapes.end(),
			 [=](const std::shared_ptr<geo::Shape> shp){return *std::shared_ptr<geo::Shape>(shp) == *shape;});
  EXPECT_EQ(*std::shared_ptr<geo::Shape>(*it), *shape); // "double dereferencing" (iterator and smart ptr)
  EXPECT_EQ(*shape->parent.lock(), *group);
}

TEST(GeoTest, GroupRayIntersectsEmpty) {

  // Intersecting a Ray with an empty group
  auto group = std::make_shared<geo::Group>();
  ray::Ray ray(math::Point(0, 0, 0), math::Vector(0, 0, 1));
  auto xs = group->local_intersects(ray);
  EXPECT_EQ(xs.size(), 0);  
}

TEST(GeoTest, GroupRayIntersectsNotEmpty) {

  // Intersecting a Ray with a nonempty group
  auto group = std::make_shared<geo::Group>();
  auto sphere1 = std::make_shared<geo::Sphere>();
  auto sphere2 = std::make_shared<geo::Sphere>();
  auto sphere3 = std::make_shared<geo::Sphere>();
  
  sphere2->transform = math::translation(0, 0, -3);
  sphere3->transform = math::translation(5, 0, 0);

  group->add_child(sphere1);
  group->add_child(sphere2);
  group->add_child(sphere3);

  ray::Ray ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto xs = group->local_intersects(ray);

  ASSERT_EQ(xs.size(), 4);
  EXPECT_EQ(*xs[0].geometry, *sphere2);
  EXPECT_EQ(*xs[1].geometry, *sphere2);
  EXPECT_EQ(*xs[2].geometry, *sphere1);
  EXPECT_EQ(*xs[3].geometry, *sphere1);
}

TEST(GeoTest, GroupTransforms) {

  // Group and child transformations are both applied
  auto group = std::make_shared<geo::Group>();
  group->transform = math::scaling(2, 2, 2);
  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::translation(5, 0, 0);
  group->add_child(sphere);

  ray::Ray ray(math::Point(10, 0, -10), math::Vector(0, 0, 1));
  auto xs = group->intersects(ray);
  EXPECT_EQ(xs.size(), 2);
}

TEST(GeoTest, GroupWorldPointToObjectPoint) {

  // Converting a point from world to object space, taking into consideration every parent objects
  auto group1 = std::make_shared<geo::Group>();
  group1->transform = math::rotation_y(M_PI / 2);
  auto group2 = std::make_shared<geo::Group>();
  group2->transform = math::scaling(2, 2, 2);
  group1->add_child(group2);
  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::translation(5, 0, 0);
  group2->add_child(sphere);

  auto point = sphere->world_to_object(math::Point(-2, 0, -10));
  ASSERT_NE(sphere->parent.lock(), nullptr);
  ASSERT_EQ(*sphere->parent.lock(), *group2);
  ASSERT_NE(group2->parent.lock(), nullptr);
  ASSERT_EQ(*group2->parent.lock(), *group1);
  ASSERT_EQ(group1->parent.lock(), nullptr);
  ASSERT_NE(sphere->parent.lock()->parent.lock(), nullptr);
  ASSERT_EQ(*sphere->parent.lock()->parent.lock(), *group1);
  EXPECT_EQ(point, math::Point(0, 0, -1));
}

TEST(GeoTest, GroupANormalVectorFromObjectToWorldSpace) {

  // Taking a normal vector in object space to convert it to world space, taking into consideration every parent objects
  auto group1 = std::make_shared<geo::Group>();
  group1->transform = math::rotation_y(M_PI / 2);
  auto group2 = std::make_shared<geo::Group>();
  group2->transform = math::scaling(1, 2, 3);
  group1->add_child(group2);
  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::translation(5, 0, 0);
  group2->add_child(sphere);

  auto normal = sphere->normal_to_world(math::Vector(sqrt(3)/3, sqrt(3)/3, sqrt(3)/3));
  EXPECT_EQ(normal, math::Vector(0.28571, 0.42857, -0.85714));
}

TEST(GeoTest, GroupFindNormalOfObject) {

  // Finding the normal on a child object, taking into consideration the transformations of the child and parent
  auto group1 = std::make_shared<geo::Group>();
  group1->transform = math::rotation_y(M_PI / 2);
  auto group2 = std::make_shared<geo::Group>();
  group2->transform = math::scaling(1, 2, 3);
  group1->add_child(group2);
  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::translation(5, 0, 0);
  group2->add_child(sphere);

  auto normal = sphere->normal_at(math::Point(1.7321, 1.1547, -5.5774), geo::Intersection(INFINITY, sphere.get()));
  EXPECT_EQ(normal, math::Vector(0.285704, 0.428543, -0.857161));
}

TEST(GeoTest, BaseReflection){

  // Reflect a vector at 45 degrees
  auto vec = math::Vector(1.0, -1.0, 0.0);
  auto normal = math::Vector(0.0, 1.0, 0.0);

  auto ref = geo::reflect(vec, normal);
  EXPECT_EQ(ref, math::Vector(1.0, 1.0, 0.0));

  // Reflect a vector off a slanted surface
  vec = math::Vector(0.0, -1.0, 0.0);\
  normal = math::Vector(sqrt(2) / 2, sqrt(2) / 2, 0.0);

  ref = geo::reflect(vec, normal);
  EXPECT_EQ(ref, math::Vector(1.0, 0.0, 0.0));
}

TEST(GeoTest, ShapeMaterial) {

  // The default material
  auto test_shape = std::make_shared<geo::TestShape>();
  EXPECT_EQ(test_shape->material, material::Material());

  // A Shape can be assigned a material
  auto m = material::Material();
  m.ambient = 1;
  test_shape->material = m;
  EXPECT_EQ(test_shape->material, m);
}

TEST(GeoTest, PrepareComputations) {

  // Precompute the state of an intersection
  auto r = ray::Ray(math::Point(0.0, 0.0, -5.0), math::Vector(0.0, 0.0, 1.0));
  auto shape = std::make_shared<geo::Sphere>();
  auto i = geo::Intersection(4.0, shape.get());

  auto comps = geo::prepare_computations(i, r);
  EXPECT_EQ(comps.t, i.t);
  EXPECT_EQ(comps.geometry, i.geometry);
  EXPECT_EQ(comps.point, math::Point(0.0, 0.0, -1.0));
  EXPECT_EQ(comps.eye_vector, math::Vector(0.0, 0.0, -1.0));
  EXPECT_EQ(comps.normal_vector, math::Vector(0.0, 0.0, -1.0));
}

TEST(GeoTest, InsideObjectIntersection) {

  // The hit, when an intersection occurs on the outside
  auto r = ray::Ray(math::Point(0.0, 0.0, -5.0), math::Vector(0.0, 0.0, 1.0));
  auto shape = std::make_shared<geo::Sphere>();
  auto ix = geo::Intersection(4, shape.get());

  auto comps = geo::prepare_computations(ix, r);
  EXPECT_FALSE(comps.inside);

  // The hit, when an intersection occurs on the inside
  r = ray::Ray(math::Point(0.0, 0.0, 0.0), math::Vector(0.0, 0.0, 1.0));
  ix = geo::Intersection(1.0, shape.get());
  comps = geo::prepare_computations(ix, r);
  EXPECT_EQ(comps.point, math::Point(0.0, 0.0, 1.0));
  EXPECT_EQ(comps.eye_vector, math::Vector(0.0, 0.0, -1.0));
  EXPECT_TRUE(comps.inside);
  // normal is "inverted"
  EXPECT_EQ(comps.normal_vector, math::Vector(0.0, 0.0, -1.0));
}

TEST(GeoTest, OverPoint) {

  // The hit should offset the point
  auto r = ray::Ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto shape = std::make_shared<geo::Sphere>();
  shape->transform = math::translation(0, 0, 1);
  auto ix = geo::Intersection(5, shape.get());
  auto comps = geo::prepare_computations(ix, r);
  EXPECT_TRUE(comps.over_point.z < - math::EPSILON / 2);
  EXPECT_TRUE(comps.point.z > comps.over_point.z);
}

TEST(GeoTest, ReflectionPrepareComputations) {

  // prepare_computations precomputes the reflect_vector
  auto shape = std::make_shared<geo::Plane>();
  auto r = ray::Ray(math::Point(0, 1, -1), math::Vector(0, -sqrt(2) / 2, sqrt(2) / 2));
  auto i = geo::Intersection(sqrt(2), shape.get());
  auto comps = geo::prepare_computations(i, r);
  EXPECT_EQ(comps.reflect_vector, math::Vector(0, sqrt(2) / 2, sqrt(2) / 2)); // reflect vector bounces on the plane at 45
}
TEST(GeoTest, RefractionN1AndN2) {

  // Finding n1 and n2 at different intersections
  auto a = geo::build_glass_sphere();
  a->transform = math::scaling(2, 2, 2);
  a->material.refractive_index = 1.5;

  auto b = geo::build_glass_sphere();
  b->transform = math::translation(0, 0, -0.25);
  b->material.refractive_index = 2.0;

  auto c = geo::build_glass_sphere();
  c->transform = math::translation(0, 0, 0.25);
  c->material.refractive_index = 2.5;

  auto r = ray::Ray(math::Point(0, 0, -4), math::Vector(0, 0, 1));
  auto xs = geo::Intersections {geo::Intersection(2, a.get()),
				geo::Intersection(2.75, b.get()),
				geo::Intersection(3.25, c.get()),
				geo::Intersection(4.75, b.get()),
				geo::Intersection(5.25, c.get()),
				geo::Intersection(6, a.get()),
  };

  std::vector<double> n1_results {1.0, 1.5, 2.0, 2.5, 2.5, 1.5};

  std::vector<double> n2_results {1.5, 2.0, 2.5, 2.5, 1.5, 1.0};
  
  for (auto i = 0; i < 6; i++) {
    auto comps = geo::prepare_computations(xs.at(i), r, xs);
    EXPECT_EQ(comps.n1, n1_results.at(i));
    EXPECT_EQ(comps.n2, n2_results.at(i));
  }
}

TEST(GeoTest, ComputationsUnderPoint) {

  // Computes under_point, which lies beneath intersected surface
  auto r = ray::Ray(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto s = geo::build_glass_sphere();
  s->transform = math::translation(0, 0, 1);
  auto i = geo::Intersection(5, s.get());
  auto xs = geo::Intersections {i};
  auto comps = geo::prepare_computations(i, r, xs);

  EXPECT_GT(comps.under_point.z, math::EPSILON / 2);
  EXPECT_LT(comps.point.z, comps.under_point.z);
}

TEST(GeoTest, FresnelEffectTotalInternalReflection) {

  // The Schlick approximation under total internal reflection
  auto shape = geo::build_glass_sphere();
  auto r = ray::Ray(math::Point(0, 0, sqrt(2)/2), math::Vector(0, 1, 0));
  auto xs = geo::Intersections{geo::Intersection(-sqrt(2)/2, shape.get()), geo::Intersection(sqrt(2)/2, shape.get())};
  auto comps = geo::prepare_computations(xs[1], r, xs);
  auto reflectance = comps.schlick();
  EXPECT_EQ(reflectance, 1);
}

TEST(GeoTest, FresnelEffectPerpendicularRayReflectance) {

  // The Schlick approximation with a perpendicular ray is small
  auto shape = geo::build_glass_sphere();
  auto r = ray::Ray(math::Point(0, 0, 0), math::Vector(0, 1, 0));
  auto xs = geo::Intersections{geo::Intersection(-1, shape.get()), geo::Intersection(1, shape.get())};
  auto comps = geo::prepare_computations(xs[1], r, xs);
  auto reflectance = comps.schlick();
  EXPECT_TRUE(math::almost_equal(reflectance, 0.04));
}

TEST(GeoTest, FresnelEffectSmallAngleAndN1GTN2) {

  // like looking across a lake to the far shore
  // The Schlick approximation with small angle and n2 > n1
  auto shape = geo::build_glass_sphere();
  auto r = ray::Ray(math::Point(0, 0.99, -2), math::Vector(0, 0, 1));
  auto xs = geo::Intersections{geo::Intersection(1.8589, shape.get())};
  auto comps = geo::prepare_computations(xs[0], r, xs);
  auto reflectance = comps.schlick();
  EXPECT_TRUE(math::almost_equal(reflectance, 0.48873));
}

TEST(GeoTest, BoundingBoxDefaultStructure) {

  // A BoundingBox is a structure holding the minumum and maximum coordinates of a bounding box
  // We store those coordinates as Points
  auto bounds = geo::BoundingBox();

  EXPECT_TRUE(math::is_point(bounds.minimum));
  EXPECT_TRUE(math::is_point(bounds.maximum));

  EXPECT_EQ(bounds.minimum.x, INFINITY);
  EXPECT_EQ(bounds.minimum.y, INFINITY);
  EXPECT_EQ(bounds.minimum.z, INFINITY);
  
  EXPECT_EQ(bounds.maximum.x, -INFINITY);
  EXPECT_EQ(bounds.maximum.y, -INFINITY);
  EXPECT_EQ(bounds.maximum.z, -INFINITY);
}

TEST(GeoTest, BoundingBoxConstructor) {

  // A BoundingBox can be constructed by passing two Points
  auto bounds = geo::BoundingBox(math::Point(3.8, 1.4, 2.1), math::Point(0.2, 1, 9.4));
  EXPECT_EQ(bounds.minimum, math::Point(3.8, 1.4, 2.1));
  EXPECT_EQ(bounds.maximum, math::Point(0.2, 1, 9.4));
}

TEST(GeoTest, BoundingBoxIncludePoint) {

  auto box = geo::BoundingBox();
  auto point1 = math::Point(-5, 2, 0);
  auto point2 = math::Point(7, 0, -3);

  box.include(point1);
  box.include(point2);
  EXPECT_EQ(box.minimum, math::Point(-5, 0, -3));
  EXPECT_EQ(box.maximum, math::Point(7, 2, 0));
}

TEST(GeoTest, TestShapeBoundingBox) {

  // A TestShape returns the default BoundingBox
  auto test_shape = std::make_shared<geo::TestShape>();
  auto bounds = test_shape->get_bounds();

  EXPECT_EQ(bounds.minimum, math::Point(-1, -1, -1));
  EXPECT_EQ(bounds.maximum, math::Point(1, 1, 1));  
}

TEST(GeoTest, SphereBoundingBox) {

  // An untransformed Sphere extends from -1 to 1 on x, y and z axis
  auto sphere = std::make_shared<geo::Sphere>();
  //auto bounds = sphere->get_bounds();

  EXPECT_EQ(sphere->bounds.minimum, math::Point(-1, -1, -1));
  EXPECT_EQ(sphere->bounds.maximum, math::Point(1, 1, 1));
}

TEST(GeoTest, PlaneBoundingBox) {
  
  // An untransformed Plane stretches to  infinity on x and z, 0 on y
  auto plane = std::make_shared<geo::Plane>();

  EXPECT_EQ(plane->bounds.minimum.x, -INFINITY);
  EXPECT_EQ(plane->bounds.minimum.y, 0);
  EXPECT_EQ(plane->bounds.minimum.z, -INFINITY);
  
  EXPECT_EQ(plane->bounds.maximum.x, INFINITY);
  EXPECT_EQ(plane->bounds.maximum.y, 0);  
  EXPECT_EQ(plane->bounds.maximum.z, INFINITY);
}

TEST(GeoTest, CubeBoundingBox) {
  
  // An untransformed Cube extends from -1 to 1 on x, y and z
  auto cube = std::make_shared<geo::Cube>();

  EXPECT_EQ(cube->bounds.minimum, math::Point(-1, -1, -1));
  EXPECT_EQ(cube->bounds.maximum, math::Point(1, 1, 1));  
}

TEST(GeoTest, CylinderBoundingBox) {
  
  // An untransformed untruncated Cylinder is infinite on y, and extends from -1 to 1 on x and z
  auto cylinder = std::make_shared<geo::Cylinder>();

  EXPECT_EQ(cylinder->bounds.minimum.x, -1);
  EXPECT_EQ(cylinder->bounds.minimum.y, -INFINITY);
  EXPECT_EQ(cylinder->bounds.minimum.z, -1);

  EXPECT_EQ(cylinder->bounds.maximum.x, 1);
  EXPECT_EQ(cylinder->bounds.maximum.y, INFINITY);
  EXPECT_EQ(cylinder->bounds.maximum.z, 1);  
}

TEST(GeoTest, TruncatedCylinderBoundingBox) {
  
  // An untransformed untruncated Truncated_Cylinder is infinite on y, and extends from -1 to 1 on x and z
  auto truncated_cylinder = std::make_shared<geo::Cylinder>();
  // TODO: Add member func to set min and max, updating the boundingbox
  truncated_cylinder->set_minimum(-5);
  truncated_cylinder->set_maximum(3);
  auto bounds = truncated_cylinder->bounds;//truncated_cylinder->get_bounds();

  EXPECT_EQ(bounds.minimum.x, -1);
  EXPECT_EQ(bounds.minimum.y, -5);
  EXPECT_EQ(bounds.minimum.z, -1);

  EXPECT_EQ(bounds.maximum.x, 1);
  EXPECT_EQ(bounds.maximum.y, 3);
  EXPECT_EQ(bounds.maximum.z, 1);  
}

TEST(GeoTest, ConeBoundingBox) {
  
  // An untransformed untruncated Cone is infinite on y, and its radius (x, z) extends from -y to y
  auto cone = std::make_shared<geo::DoubleCone>();
  auto bounds = cone->get_bounds();

  EXPECT_EQ(bounds.minimum.x, -INFINITY);
  EXPECT_EQ(bounds.minimum.y, -INFINITY);
  EXPECT_EQ(bounds.minimum.z, -INFINITY);

  EXPECT_EQ(bounds.maximum.x, INFINITY);
  EXPECT_EQ(bounds.maximum.y, INFINITY);
  EXPECT_EQ(bounds.maximum.z, INFINITY);  
}

TEST(GeoTest, TruncatedConeBoundingBox) {
  
  // An untransformed  truncated Cone is infinite on y, and its radius (x, z) extends from -y to y
  auto truncated_cone = std::make_shared<geo::DoubleCone>();
  truncated_cone->set_minimum(-5);
  truncated_cone->set_maximum(3);
  auto bounds = truncated_cone->bounds;

  EXPECT_EQ(bounds.minimum, math::Point(-5, -5, -5));
  EXPECT_EQ(bounds.maximum, math::Point(5, 3, 5));
}

TEST(GeoTest, TriangleBoundingBox) {

  // Just find smallest and largest x, y, and z components
  math::Point p1(-3, 7, 2);
  math::Point p2(6, 2, -4);
  math::Point p3(2, -1, -1);
  auto triangle = std::make_shared<geo::Triangle>(p1, p2, p3);
  auto bounds = triangle->bounds;

  EXPECT_EQ(bounds.minimum, math::Point(-3, -1, -4));
  EXPECT_EQ(bounds.maximum, math::Point(6, 7, 2));
}

TEST(GeoTest, AddingBoundingBoxesTogether) {

  auto box1 = geo::BoundingBox(math::Point(-5, -2, 0), math::Point(7, 4, 4));
  auto box2 = geo::BoundingBox(math::Point(8, -7, -2), math::Point(14, 2, 8));
  box1.add(box2);
  
  EXPECT_EQ(box1.minimum, math::Point(-5, -7, -2));
  EXPECT_EQ(box1.maximum, math::Point(14, 4, 8));
}

TEST(GeoTest, BoundingBoxContainsPointFunc) {

  struct TestInput {
    math::Point point;
    bool result;

    TestInput(const math::Point& p, const bool r) : point {p}, result {r} {}
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(5, -2, 0), true),
    TestInput(math::Point(11, 4, 7), true),
    TestInput(math::Point(8, 1, 3), true),
    TestInput(math::Point(3, 0, 3), true),
    TestInput(math::Point(8, -4, 3), false),
    TestInput(math::Point(8, 1, -1), false),
    TestInput(math::Point(13, 1, 3), false),
    TestInput(math::Point(8, 5, 3), false),
    TestInput(math::Point(8, 1, 8), false),      
  };
  
  auto box = geo::BoundingBox(math::Point(-5, -2, 0), math::Point(11, 4, 7));
  for (const auto& input : inputs)
    EXPECT_EQ(box.contains(input.point), input.result);
}

TEST(GeoTest, BoundingBoxContainsAnotherBoundingBoxFunc) {

  struct TestInput {
    math::Point min;
    math::Point max;
    bool result;

    TestInput(const math::Point& mi, const math::Point& ma, const bool r)
      : min {mi}, max {ma}, result {r} {}
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(5, -2, 0), math::Point(11, 4, 7), true),
    TestInput(math::Point(6, -1, 1), math::Point(10, 3, 6), true),
    TestInput(math::Point(4, -3, -1), math::Point(10, 3, 6), false),
    TestInput(math::Point(6, -1, 1), math::Point(12, 5, 8), false),
  };

  auto box = geo::BoundingBox(math::Point(-5, -2, 0), math::Point(11, 4, 7));
  for (const auto& input : inputs)
    EXPECT_EQ(box.contains(geo::BoundingBox(input.min, input.max)), input.result);
}

TEST(GeoTest, BoundingBoxTransform) {

  geo::BoundingBox box(math::Point(-1, -1, -1), math::Point(1, 1, 1));
  auto matrix = math::rotation_x(M_PI / 4) * math::rotation_y(M_PI / 4);
  auto box2 = box.transform(matrix);
  EXPECT_EQ(box2.minimum, math::Point(-1.41421, -1.70711, -1.70711));
  EXPECT_EQ(box2.maximum, math::Point(1.41421, 1.70711, 1.70711));
}

TEST(GeoTest, BoundingBoxInParentSpace) {

  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::translation(1, -3, 5) * math::scaling(0.5, 2, 4);
  auto box = sphere->get_parent_space_bounds();
  EXPECT_EQ(box.minimum, math::Point(0.5, -5, 1));
  EXPECT_EQ(box.maximum, math::Point(1.5, -1, 9));
}

TEST(GeoTest, BoundingBoxGroup) {

  // A Group has a BoundingBox that contains its children
  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::translation(2, 5, -3) * math::scaling(2, 2, 2);

  auto cyl = std::make_shared<geo::Cylinder>();
  cyl->set_minimum(-2);
  cyl->set_maximum(2);
  cyl->transform = math::translation(-4, -1, 4) * math::scaling(0.5, 1, 0.5);

  auto group = std::make_shared<geo::Group>();
  group->add_child(sphere);
  group->add_child(cyl);

  auto box = group->bounds;//nds();
  EXPECT_EQ(box.minimum, math::Point(-4.5, -3, -5));
  EXPECT_EQ(box.maximum, math::Point(4, 7, 4.5));
}

TEST(GeoTest, BoundingBoxIntersectsRay) {

  struct TestInput {
    math::Tuple origin;
    math::Tuple direction;
    bool result;

    TestInput(const math::Tuple& o, const math::Tuple& d, const bool r)
      : origin {o}, direction {d}, result {r} {};
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(5, 0.5, 0), math::Vector(-1, 0, 0), true),
    TestInput(math::Point(-5, 0.5, 0), math::Vector(1, 0, 0), true),
    TestInput(math::Point(0.5, 5, 0), math::Vector(0, -1, 0), true),
    TestInput(math::Point(0.5, -5, 0), math::Vector(0, 1, 0), true),
    TestInput(math::Point(0.5, 0, 5), math::Vector(0, 0, -1), true),
    TestInput(math::Point(0.5, 0, -5), math::Vector(0, 0, 1), true),
    TestInput(math::Point(0, 0.5, 0), math::Vector(0, 0, 1), true),
    TestInput(math::Point(-2, 0, 0), math::Vector(2, 4, 6), false),
    TestInput(math::Point(0, -2, 0), math::Vector(6, 2, 4), false),
    TestInput(math::Point(0, 0, -2), math::Vector(4, 6, 2), false),
    TestInput(math::Point(2, 0, 2), math::Vector(0, 0, -1), false),
    TestInput(math::Point(0, 2, 2), math::Vector(0, -1, 0), false),
    TestInput(math::Point(2, 2, 0), math::Vector(-1, 0, 0), false),      
  };
  
  geo::BoundingBox box(math::Point(-1, -1, -1), math::Point(1, 1, 1));
  for (const auto& input : inputs) {
    auto r = ray::Ray(input.origin, math::normalize(input.direction));
    bool result = box.intersects(r);
    EXPECT_EQ(result, input.result);
  }
}

TEST(GeoTest, NonCubicBoundingBoxIntersectsRay) {

  struct TestInput {
    math::Tuple origin;
    math::Tuple direction;
    bool result;

    TestInput(const math::Tuple& o, const math::Tuple& d, const bool r)
      : origin {o}, direction {d}, result {r} {};
  };

  std::vector<TestInput> inputs {
    TestInput(math::Point(15, 1, 2), math::Vector(-1, 0, 0), true),
    TestInput(math::Point(-5, -1, 4), math::Vector(1, 0, 0), true),
    TestInput(math::Point(7, 6, 5), math::Vector(0, -1, 0), true),
    TestInput(math::Point(9, -5, 6), math::Vector(0, 1, 0), true),
    TestInput(math::Point(8, 2, 12), math::Vector(0, 0, -1), true),
    TestInput(math::Point(6, 0, -5), math::Vector(0, 0, 1), true),
    TestInput(math::Point(8, 1, 3.5), math::Vector(0, 0, 1), true),
    TestInput(math::Point(9, -1, -8), math::Vector(2, 4, 6), false),
    TestInput(math::Point(8, 3, -4), math::Vector(6, 2, 4), false),
    TestInput(math::Point(9, -1, -2), math::Vector(4, 6, 2), false),
    TestInput(math::Point(4, 0, 9), math::Vector(0, 0, -1), false),
    TestInput(math::Point(8, 6, -1), math::Vector(0, -1, 0), false),
    TestInput(math::Point(12, 5, 4), math::Vector(-1, 0, 0), false),
  };
  
  geo::BoundingBox box(math::Point(5, -2, 0), math::Point(11, 4, 7));
  for (const auto& input : inputs) {
    ray::Ray r(input.origin, math::normalize(input.direction));
    EXPECT_EQ(box.intersects(r), input.result);
  }

}

TEST(GeoTest, GroupIntersectsIfBoundingBoxMissedNoTestChildren) {

  auto child = std::make_shared<geo::TestShape>();
  auto group = std::make_shared<geo::Group>();
  std::cout << group->bounds.minimum << ", " << group->bounds.maximum << std::endl;
  group->add_child(child);
  std::cout << group->bounds.minimum << ", " << group->bounds.maximum << std::endl;
  ray::Ray r(math::Point(0, 0, -5), math::Vector(0, 1, 0));
  auto xs = group->intersects(r);
  EXPECT_EQ(child->saved_ray, ray::Ray(math::Point(0, 0, 0), math::Vector(0, 0, 0)));
}

TEST(GeoTest, GroupIntersectsIfBoundingBoxHitTestChildren) {
  
  auto child = std::make_shared<geo::TestShape>();
  auto group = std::make_shared<geo::Group>();
  group->add_child(child);
  ray::Ray r(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto xs = group->intersects(r);
  EXPECT_NE(child->saved_ray, ray::Ray(math::Point(0, 0, 0), math::Vector(0, 0, 0)));  
}

TEST(GeoTest, BoundingBoxSplitCube) {

  geo::BoundingBox box(math::Point(-1, -4, -5), math::Point(9, 6, 5));
  auto [left, right] = box.split();
  EXPECT_EQ(left.minimum, math::Point(-1, -4, -5));
  EXPECT_EQ(left.maximum, math::Point(4, 6, 5));
  EXPECT_EQ(right.minimum, math::Point(4, -4, -5));
  EXPECT_EQ(right.maximum, math::Point(9, 6, 5));  
}

TEST(GeoTest, BoundingBoxSplitXWide) {
  geo::BoundingBox box(math::Point(-1, -2, -3), math::Point(9, 5.5, 3));
  auto [left, right] = box.split();
  EXPECT_EQ(left.minimum, math::Point(-1, -2, -3));
  EXPECT_EQ(left.maximum, math::Point(4, 5.5, 3));
  EXPECT_EQ(right.minimum, math::Point(4, -2, -3));
  EXPECT_EQ(right.maximum, math::Point(9, 5.5, 3));  
}

TEST(GeoTest, BoundingBoxSplitYWide) {
  geo::BoundingBox box(math::Point(-1, -2, -3), math::Point(5, 8, 3));
  auto [left, right] = box.split();
  EXPECT_EQ(left.minimum, math::Point(-1, -2, -3));
  EXPECT_EQ(left.maximum, math::Point(5, 3, 3));
  EXPECT_EQ(right.minimum, math::Point(-1, 3, -3));
  EXPECT_EQ(right.maximum, math::Point(5, 8, 3));
}

TEST(GeoTest, BoundingBoxSplitZWide) {
  geo::BoundingBox box(math::Point(-1, -2, -3), math::Point(5, 3, 7));
  auto [left, right] = box.split();
  EXPECT_EQ(left.minimum, math::Point(-1, -2, -3));
  EXPECT_EQ(left.maximum, math::Point(5, 3, 2));
  EXPECT_EQ(right.minimum, math::Point(-1, -2, 2));
  EXPECT_EQ(right.maximum, math::Point(5, 3, 7));
}

// TEST(GeoTest, GroupPartitionChildren) {

//   auto sphere1 = std::make_shared<geo::Sphere>();
//   sphere1->transform = math::translation(-2, 0, 0);
//   auto sphere2 = std::make_shared<geo::Sphere>();
//   sphere2->transform = math::translation(2, 0, 0);
//   auto sphere3 = std::make_shared<geo::Sphere>();
//   auto group = std::make_shared<geo::Group>();
//   group->add_child(sphere1);
//   group->add_child(sphere2);
//   group->add_child(sphere3);

//   auto [left, right] = group->partition_children();
//   ASSERT_EQ(group->shapes.size(), 1);
//   EXPECT_EQ(*(group->shapes[0]), *sphere3);
//   ASSERT_EQ(left.size(), 1);
//   EXPECT_EQ(*left[0], *sphere1);
//   ASSERT_EQ(right.size(), 1);
//   EXPECT_EQ(*right[0], *sphere2);
// }

// TEST(GeoTest, SubGroupFromListOfChildren) {

//   auto sphere1 = std::make_shared<geo::Sphere>();
//   auto sphere2 = std::make_shared<geo::Sphere>();
//   auto group = std::make_shared<geo::Group>();
//   group->make_subgroup(std::vector<std::shared_ptr<geo::Shape>>{sphere1, sphere2});

//   ASSERT_EQ(group->shapes.size(), 1);
//   // the only child of this Group is another Group
//   geo::Group* subgroupptr = dynamic_cast<geo::Group*>(group->shapes[0].get());
//   ASSERT_NE(subgroupptr, nullptr);
//   ASSERT_EQ(subgroupptr->shapes.size(), 2);
//   EXPECT_EQ(*subgroupptr->shapes[0], *sphere1);
//   EXPECT_EQ(*subgroupptr->shapes[1], *sphere2);
// }

TEST(GeoTest, SubdividingPrimitiveDoesNothing) {

  auto sphere = std::make_shared<geo::Sphere>();
  sphere->divide(1);
  // sphere stays a Sphere
  EXPECT_NE(dynamic_cast<geo::Sphere*>(sphere.get()), nullptr);
}

TEST(GeoTest, SubdividingGroupPartitionsItsChildren) {

  auto sphere1 = std::make_shared<geo::Sphere>();
  sphere1->transform = math::translation(-2, -2, 0);
  auto sphere2 = std::make_shared<geo::Sphere>();
  sphere2->transform = math::translation(-2, 2, 0);
  auto sphere3 = std::make_shared<geo::Sphere>();
  sphere3->transform = math::scaling(4, 4, 4);
  auto group = std::make_shared<geo::Group>();
  group->add_child(sphere1);
  group->add_child(sphere2);
  group->add_child(sphere3);

  group->divide(1);
  EXPECT_EQ(*group->shapes[0], *sphere3);
  //subgroup is a Group
  auto subgroup = dynamic_cast<geo::Group*>(group->shapes[1].get());
  ASSERT_NE(subgroup, nullptr);
  ASSERT_EQ(subgroup->shapes.size(), 2);
  // subgroup contains two sub-subgroups
  auto subsubgroup1 = dynamic_cast<geo::Group*>(subgroup->shapes[0].get());
  auto subsubgroup2 = dynamic_cast<geo::Group*>(subgroup->shapes[1].get());
  ASSERT_NE(subsubgroup1, nullptr);
  ASSERT_NE(subsubgroup2, nullptr);
  ASSERT_EQ(subsubgroup1->shapes.size(), 1);
  ASSERT_EQ(subsubgroup2->shapes.size(), 1);
  EXPECT_EQ(*subsubgroup1->shapes[0], *sphere1);
  EXPECT_EQ(*subsubgroup2->shapes[0], *sphere2);
}

TEST(GeoTest, SubdividingGroupWithTooFewChildren) {

  auto sphere1 = std::make_shared<geo::Sphere>();
  sphere1->transform = math::translation(-2, 0, 0);
  auto sphere2 = std::make_shared<geo::Sphere>();
  sphere2->transform = math::translation(2, 1, 0);
  auto sphere3 = std::make_shared<geo::Sphere>();
  sphere3->transform = math::translation(2, -1, 0);
  auto subgroup = std::make_shared<geo::Group>();
  subgroup->add_child(sphere1);
  subgroup->add_child(sphere2);
  subgroup->add_child(sphere3);
  auto sphere4 = std::make_shared<geo::Sphere>();
  auto group = std::make_shared<geo::Group>();
  group->add_child(subgroup);
  group->add_child(sphere4);
  group->divide(3);

  EXPECT_EQ(*group->shapes[0], *subgroup);
  EXPECT_EQ(*group->shapes[1], *sphere4);
  EXPECT_EQ(subgroup->shapes.size(), 2);
  auto subsubgroup1 = dynamic_cast<geo::Group*>(subgroup->shapes[0].get());
  ASSERT_NE(subsubgroup1, nullptr);
  ASSERT_EQ(subsubgroup1->shapes.size(), 1);
  EXPECT_EQ(*subsubgroup1->shapes[0], *sphere1);
  auto subsubgroup2 = dynamic_cast<geo::Group*>(subgroup->shapes[1].get());
  ASSERT_NE(subsubgroup2, nullptr);
  ASSERT_EQ(subsubgroup2->shapes.size(), 2);
  EXPECT_EQ(*subsubgroup2->shapes[0], *sphere2);
  EXPECT_EQ(*subsubgroup2->shapes[1], *sphere3);
}

