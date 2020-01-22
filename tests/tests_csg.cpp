
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "geo.hpp"
#include "csg.hpp"


TEST(GeoTest, CSGShapeCreation) {

  // A CSG Shape is the result of a set operation on two Shapes
  auto sphere = std::make_shared<geo::Sphere>();
  auto cube = std::make_shared<geo::Cube>();
  auto c = sphere | cube; // union

  EXPECT_EQ(c->operation, "union");
  EXPECT_EQ(*(c->left), *sphere);
  EXPECT_EQ(*(c->right), *cube);
  EXPECT_EQ(*(sphere->parent.lock()), *c);
  EXPECT_EQ(*(cube->parent.lock()), *c);  
}

TEST(GeoTest, CSGShapeIntersectionsRules) {

  struct TestInput {
    std::string operation;
    bool left_hit;
    bool in_left;
    bool in_right;
    bool result;
    
    TestInput(const std::string op, const bool lhit, bool inl, bool inr, bool r)
      : operation {op}, left_hit {lhit}, in_left {inl}, in_right {inr}, result {r} {}
  };

  std::vector<TestInput> test_inputs {
    // A CSG union preserves all intersection on the exterior of both Shapes
    TestInput("union", true, true, true, false),
    TestInput("union", true, true, false, true), 
    TestInput("union", true, false, true, false),
    TestInput("union", true, false, false, true),
    TestInput("union", false, true, true, false),
    TestInput("union", false, true, false, false), 
    TestInput("union", false, false, true, true),
    TestInput("union", false, false, false, true),
      // A CSG intersect preserves all intersections where both Shape overlap
    TestInput("intersect", true, true, true, true),
    TestInput("intersect", true, true, false, false), 
    TestInput("intersect", true, false, true, true),
    TestInput("intersect", true, false, false, false),
    TestInput("intersect", false, true, true, true),
    TestInput("intersect", false, true, false, true),
    TestInput("intersect", false, false, true, false),
    TestInput("intersect", false, false, false, false),
      // A CSG difference preserves all intersections not exclusively inside the right object
    TestInput("difference", true, true, true, false),
    TestInput("difference", true, true, false, true), 
    TestInput("difference", true, false, true, false),
    TestInput("difference", true, false, false, true),
    TestInput("difference", false, true, true, true),
    TestInput("difference", false, true, false, true),
    TestInput("difference", false, false, true, false),
    TestInput("difference", false, false, false, false),      
   };

  for (const auto& input : test_inputs)
    EXPECT_EQ(geo::intersection_allowed(input.operation, input.left_hit, input.in_left, input.in_right), input.result);
}

