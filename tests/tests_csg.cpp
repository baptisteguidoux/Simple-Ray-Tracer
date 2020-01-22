
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

TEST(GeoTest, CSGShapeUnionIntersectionAllowed) {

  // A CSG Union preserves all intersection on the exterior of both Shapes

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
    TestInput("union", true, true, true, false),
    TestInput("union", true, true, false, true), 
    TestInput("union", true, false, true, false),
    TestInput("union", true, false, false, true),
    TestInput("union", false, true, true, false),
    TestInput("union", false, true, false, false), 
    TestInput("union", false, false, true, true),
    TestInput("union", false, false, false, true),
   };

  for (const auto& input : test_inputs)
    EXPECT_EQ(geo::intersection_allowed(input.operation, input.left_hit, input.in_left, input.in_right), input.result);
}

