
#include <memory>
#include <cmath>

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

TEST(GeoTest, Hits) {

  // The hit when all intersections have a positive t
  auto s = std::make_shared<geo::Sphere>();
  auto inter1 = geo::Intersection(1.0, s);
  auto inter2 = geo::Intersection(2.0, s);
  auto xss = geo::Intersections {inter1, inter2};

  auto h = geo::hit(xss);
  EXPECT_EQ(h, inter1);

  // The hit, when some intersections have a negative t
  auto inter3 = geo::Intersection(-1.0, s);
  auto inter4 = geo::Intersection(2.0, s);
  xss = geo::Intersections {inter3, inter4};

  auto h2 = geo::hit(xss);
  EXPECT_EQ(h2, inter4);

  // The hit when all intersections have a negative t
  auto inter5 = geo::Intersection(-2.0, s);
  auto inter6 = geo::Intersection(-1.0, s);
  xss = geo::Intersections {inter5, inter6};

  auto h3 = geo::hit(xss);
  EXPECT_EQ(h3, std::nullopt); // std::optional void result

  // The hit is always the lowest nonegative intersection
  auto inter7 = geo::Intersection(5.0, s);
  auto inter8 = geo::Intersection(7.0, s);
  auto inter9 = geo::Intersection(-3.0, s);
  auto inter10 = geo::Intersection(2.0, s);
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
  auto normal1 = sphere->normal_at(math::Point(1.0, 0.0, 0.0));
  EXPECT_EQ(normal1, math::Vector(1.0, 0.0, 0.0));

  // Normal on a sphere at a point on the y axis
  auto normal2 = sphere->normal_at(math::Point(0.0, 1.0, 0.0));
  EXPECT_EQ(normal2, math::Vector(0.0, 1.0, 0.0));

  // Normal on a sphere at a point on the z axis
  auto normal3 = sphere->normal_at(math::Point(0.0, 0.0, 1.0));
  EXPECT_EQ(normal3, math::Vector(0.0, 0.0, 1.0));

  // The normal on a sphere at a nonaxial point
  auto normal4 = sphere->normal_at(math::Point(sqrt(3) / 3, sqrt(3) / 3, sqrt(3) / 3));
  EXPECT_EQ(normal4, math::Vector(sqrt(3) / 3, sqrt(3) / 3, sqrt(3) / 3));

  // A surface normal should always be normalized
  EXPECT_EQ(normal4, math::normalize(normal4));

  // Computing the normal on a translated shape
  auto shape = std::make_shared<geo::TestShape>();
  shape->transform = math::translation(0, 1, 0);
  auto normal7 = shape->normal_at(math::Point(0, 1.70711, -0.70711));
  EXPECT_EQ(normal7, math::Vector(0, 0.70711, -0.70711));

  // Computing the normal on a transformed shape
  shape = std::make_shared<geo::TestShape>();
  shape->transform = math::scaling(1, 0.5, 1) * math::rotation_z(M_PI / 5);
  auto normal8 = shape->normal_at(math::Point(0, sqrt(2) / 2, -sqrt(2) / 2));
  EXPECT_EQ(normal8, math::Vector(0, 0.97014, -0.24254));
}

TEST(GeoTest, NormalOnPlane) {

  // The normal of a plane is constant everywhere
  auto plane = std::make_shared<geo::Plane>();

  // Check it is subclass of Shape
  EXPECT_NE(dynamic_cast<geo::Shape*>(plane.get()), nullptr);
  
  auto norm1 = plane->local_normal_at(math::Point(0, 0, 0));
  auto norm2 = plane->local_normal_at(math::Point(10, 0, -10));
  auto norm3 = plane->local_normal_at(math::Point(-5, 0, 150));

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
    auto normal = c->local_normal_at(p);
    EXPECT_EQ(normal, input.normal);
  }
 
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
  
  EXPECT_EQ(cyl->local_normal_at(math::Point(1, 0, 0)), math::Vector(1, 0, 0));
  EXPECT_EQ(cyl->local_normal_at(math::Point(0, 5, -1)), math::Vector(0, 0, -1));
  EXPECT_EQ(cyl->local_normal_at(math::Point(0, -2, 1)), math::Vector(0, 0, 1));
  EXPECT_EQ(cyl->local_normal_at(math::Point(-1, -1, 0)), math::Vector(-1, 0, 0));
}

TEST(GeoTest, CylinderMaximumMinimumBounds) {

  // The default minimum and maximum for a cylinder
  auto cyl = geo::Cylinder();
  EXPECT_EQ(cyl.minimum, -INFINITY);
  EXPECT_EQ(cyl.maximum, INFINITY);
}

TEST(GeoTest, CylinderIntersectingTruncatedCylinder) {

  // Intersecting a constrained cylinder
  auto cyl = geo::Cylinder();
  cyl.minimum = 1;
  cyl.maximum = 2;

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
    auto xs = cyl.local_intersects(r);
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
    auto vec = cyl->local_normal_at(input.point);
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

TEST(GeoTest, GroupShape) {

  // Creating a new group
  auto group = geo::Group();
  
  EXPECT_EQ(group.transform, math::IDENTITY_MATRIX);
  EXPECT_EQ(group.shapes.size(), 0);
}

TEST(GeoTest, ShapeHasParentAttribute) {

  // A Shape has a parent variable
  auto shape = std::make_shared<geo::TestShape>();
  
  EXPECT_EQ(shape->parent, nullptr);
}

TEST(GeoTest, GroupShapeAddChild) {

  // Adding a child to a Group
  auto group = std::make_shared<geo::Group>();
  auto shape = std::make_shared<geo::TestShape>();
  group->add_child(shape);
  
  EXPECT_NE(group->shapes.size(), 0);

  // Test group includes shape
  auto it = std::find_if(group->shapes.begin(), group->shapes.end(), [=](const std::shared_ptr<geo::Shape> shp){return *shp == *shape;});
  EXPECT_EQ(**it, *shape); // "double dereferencing" (iterator and smart ptr)
  EXPECT_EQ(*shape->parent, *group);
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
    auto norm = cone->local_normal_at(input.point);
    EXPECT_EQ(norm, input.normal);
  }

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
  auto i = geo::Intersection(4.0, shape);

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
  auto ix = geo::Intersection(4, shape);

  auto comps = geo::prepare_computations(ix, r);
  EXPECT_FALSE(comps.inside);

  // The hit, when an intersection occurs on the inside
  r = ray::Ray(math::Point(0.0, 0.0, 0.0), math::Vector(0.0, 0.0, 1.0));
  ix = geo::Intersection(1.0, shape);
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
  auto ix = geo::Intersection(5, shape);
  auto comps = geo::prepare_computations(ix, r);
  EXPECT_TRUE(comps.over_point.z < - math::EPSILON / 2);
  EXPECT_TRUE(comps.point.z > comps.over_point.z);
}

TEST(GeoTest, ReflectionPrepareComputations) {

  // prepare_computations precomputes the reflect_vector
  auto shape = std::make_shared<geo::Plane>();
  auto r = ray::Ray(math::Point(0, 1, -1), math::Vector(0, -sqrt(2) / 2, sqrt(2) / 2));
  auto i = geo::Intersection(sqrt(2), shape);
  auto comps = geo::prepare_computations(i, r);
  EXPECT_EQ(comps.reflect_vector, math::Vector(0, sqrt(2) / 2, sqrt(2) / 2)); // reflect vector bounces on the plane at 45
}
TEST(GeoTest, RefractionN1AndN2) {

  // Finding n1 and n2 at different intersections
  auto a = std::make_shared<geo::GlassSphere>();
  a->transform = math::scaling(2, 2, 2);
  a->material.refractive_index = 1.5;

  auto b = std::make_shared<geo::GlassSphere>();
  b->transform = math::translation(0, 0, -0.25);
  b->material.refractive_index = 2.0;

  auto c = std::make_shared<geo::GlassSphere>();
  c->transform = math::translation(0, 0, 0.25);
  c->material.refractive_index = 2.5;

  auto r = ray::Ray(math::Point(0, 0, -4), math::Vector(0, 0, 1));
  auto xs = geo::Intersections {geo::Intersection(2, a),
				  geo::Intersection(2.75, b),
				  geo::Intersection(3.25, c),
				  geo::Intersection(4.75, b),
				  geo::Intersection(5.25, c),
				  geo::Intersection(6, a),
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
  auto s = std::make_shared<geo::GlassSphere>();
  s->transform = math::translation(0, 0, 1);
  auto i = geo::Intersection(5, s);
  auto xs = geo::Intersections {i};
  auto comps = geo::prepare_computations(i, r, xs);

  EXPECT_GT(comps.under_point.z, math::EPSILON / 2);
  EXPECT_LT(comps.point.z, comps.under_point.z);
}

TEST(GeoTest, FresnelEffectTotalInternalReflection) {

  // The Schlick approximation under total internal reflection
  auto shape = std::make_shared<geo::GlassSphere>();
  auto r = ray::Ray(math::Point(0, 0, sqrt(2)/2), math::Vector(0, 1, 0));
  auto xs = geo::Intersections{geo::Intersection(-sqrt(2)/2, shape), geo::Intersection(sqrt(2)/2, shape)};
  auto comps = geo::prepare_computations(xs[1], r, xs);
  auto reflectance = comps.schlick();
  EXPECT_EQ(reflectance, 1);
}

TEST(GeoTest, FresnelEffectPerpendicularRayReflectance) {

  // The Schlick approximation with a perpendicular ray is small
  auto shape = std::make_shared<geo::GlassSphere>();
  auto r = ray::Ray(math::Point(0, 0, 0), math::Vector(0, 1, 0));
  auto xs = geo::Intersections{geo::Intersection(-1, shape), geo::Intersection(1, shape)};
  auto comps = geo::prepare_computations(xs[1], r, xs);
  auto reflectance = comps.schlick();
  EXPECT_TRUE(math::almost_equal(reflectance, 0.04));
}

TEST(GeoTest, FresnelEffectSmallAngleAndN1GTN2) {

  // like looking across a lake to the far shore
  // The Schlick approximation with small angle and n2 > n1
  auto shape = std::make_shared<geo::GlassSphere>();
  auto r = ray::Ray(math::Point(0, 0.99, -2), math::Vector(0, 0, 1));
  auto xs = geo::Intersections{geo::Intersection(1.8589, shape)};
  auto comps = geo::prepare_computations(xs[0], r, xs);
  auto reflectance = comps.schlick();
  EXPECT_TRUE(math::almost_equal(reflectance, 0.48873));
}

