
#include <vector>

#include <gtest/gtest.h>

#include "geo.hpp"
#include "csg.hpp"
#include "tuple.hpp"
#include "matrix.hpp"


TEST(GeoTest, CSGShapeCreation) {

  // A CSG Shape is the result of a set operation on two Shapes
  auto sphere = std::make_shared<geo::Sphere>();
  auto cube = std::make_shared<geo::Cube>();
  auto c = sphere | cube; // union

  EXPECT_EQ(c->operation, geo::SetOperation::Union);
  EXPECT_EQ(*(c->left), *sphere);
  EXPECT_EQ(*(c->right), *cube);
  EXPECT_EQ(*(sphere->parent.lock()), *c);
  EXPECT_EQ(*(cube->parent.lock()), *c);  
}

TEST(GeoTest, CSGShapeIntersectionsRules) {

  struct TestInput {
    geo::SetOperation operation;
    bool left_hit;
    bool in_left;
    bool in_right;
    bool result;
    
    TestInput(const geo::SetOperation op, const bool lhit, bool inl, bool inr, bool r)
      : operation {op}, left_hit {lhit}, in_left {inl}, in_right {inr}, result {r} {}
  };

  std::vector<TestInput> test_inputs {
    // A CSG union preserves all intersection on the exterior of both Shapes
    TestInput(geo::SetOperation::Union, true, true, true, false),
    TestInput(geo::SetOperation::Union, true, true, false, true), 
    TestInput(geo::SetOperation::Union, true, false, true, false),
    TestInput(geo::SetOperation::Union, true, false, false, true),
    TestInput(geo::SetOperation::Union, false, true, true, false),
    TestInput(geo::SetOperation::Union, false, true, false, false), 
    TestInput(geo::SetOperation::Union, false, false, true, true),
    TestInput(geo::SetOperation::Union, false, false, false, true),
      // A CSG intersect preserves all intersections where both Shape overlap
    TestInput(geo::SetOperation::Intersection, true, true, true, true),
    TestInput(geo::SetOperation::Intersection, true, true, false, false), 
    TestInput(geo::SetOperation::Intersection, true, false, true, true),
    TestInput(geo::SetOperation::Intersection, true, false, false, false),
    TestInput(geo::SetOperation::Intersection, false, true, true, true),
    TestInput(geo::SetOperation::Intersection, false, true, false, true),
    TestInput(geo::SetOperation::Intersection, false, false, true, false),
    TestInput(geo::SetOperation::Intersection, false, false, false, false),
      // A CSG difference preserves all intersections not exclusively inside the right object
    TestInput(geo::SetOperation::Difference, true, true, true, false),
    TestInput(geo::SetOperation::Difference, true, true, false, true), 
    TestInput(geo::SetOperation::Difference, true, false, true, false),
    TestInput(geo::SetOperation::Difference, true, false, false, true),
    TestInput(geo::SetOperation::Difference, false, true, true, true),
    TestInput(geo::SetOperation::Difference, false, true, false, true),
    TestInput(geo::SetOperation::Difference, false, false, true, false),
    TestInput(geo::SetOperation::Difference, false, false, false, false),      
   };

  for (const auto& input : test_inputs)
    EXPECT_EQ(geo::intersection_allowed(input.operation, input.left_hit, input.in_left, input.in_right), input.result);
}

TEST(GeoTest, CSGIncludesShape) {

  auto sphere = std::make_shared<geo::Sphere>();
  auto cube = std::make_shared<geo::Cube>();
  auto plane = std::make_shared<geo::Plane>();
  auto cyl = std::make_shared<geo::Cylinder>();

  auto subunion = sphere | cube;
  auto union_ = subunion | plane;

  EXPECT_TRUE(union_->includes(subunion.get()));
  EXPECT_TRUE(union_->includes(sphere.get()));
  EXPECT_TRUE(union_->includes(cube.get()));
  EXPECT_TRUE(union_->includes(plane.get()));
  EXPECT_FALSE(union_->includes(cyl.get()));
}

TEST(GeoTest, FilteringIntersectionsList) {

  auto sphere = std::make_shared<geo::Sphere>();
  auto cube = std::make_shared<geo::Cube>();

  geo::Intersections ixs{
    geo::Intersection(1, sphere.get()), geo::Intersection(2, cube.get()),
    geo::Intersection(3, sphere.get()), geo::Intersection(4, cube.get())
  };
  
  auto csg1 = sphere | cube;
  auto result1 = csg1->filter_intersections(ixs);
  ASSERT_EQ(result1.size(), 2);
  EXPECT_EQ(result1[0], ixs[0]);
  EXPECT_EQ(result1[1], ixs[3]);  

  auto csg2 = sphere & cube;
  auto result2 = csg2->filter_intersections(ixs);
  ASSERT_EQ(result2.size(), 2);
  EXPECT_EQ(result2[0], ixs[1]);
  EXPECT_EQ(result2[1], ixs[2]);

  auto csg3 = sphere - cube;
  auto result3 = csg3->filter_intersections(ixs);
  ASSERT_EQ(result3.size(), 2);
  EXPECT_EQ(result3[0], ixs[0]);
  EXPECT_EQ(result3[1], ixs[1]);  
}

TEST(GeoTest, CSGShapeRayIntersections) {

  // Ray misses CSG Shape
  auto csg1 = std::make_shared<geo::Sphere>() | std::make_shared<geo::Cube>();
  ray::Ray r1(math::Point(0, 2, -5), math::Vector(0, 0, 1));
  auto xs1 = csg1->local_intersects(r1);
  EXPECT_EQ(xs1.size(), 0);

  // Ray hist CSG Shape
  auto sphere1 = std::make_shared<geo::Sphere>();
  auto sphere2 = std::make_shared<geo::Sphere>();
  sphere2->transform = math::translation(0, 0, 0.5);
  auto csg2 = sphere1 | sphere2;
  ray::Ray r2(math::Point(0, 0, -5), math::Vector(0, 0, 1));
  auto xs2 = csg2->local_intersects(r2);
  ASSERT_EQ(xs2.size(), 2);
  EXPECT_EQ(xs2[0].t, 4);
  EXPECT_EQ(*(xs2[0].geometry), *sphere1);
  EXPECT_EQ(xs2[1].t, 6.5);
  EXPECT_EQ(*(xs2[1].geometry), *sphere2);
}

