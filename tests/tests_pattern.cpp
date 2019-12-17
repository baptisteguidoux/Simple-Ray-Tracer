
#include "memory"

#include <gtest/gtest.h>

#include "pattern.hpp"
#include "geo.hpp"
#include "light.hpp"


TEST(PatternTest, TestPattern) {

  // It can be cast to a Pattern (abstract class)
  auto test_pattern = std::make_shared<pattern::TestPattern>();
  EXPECT_NE(dynamic_cast<pattern::Pattern*>(test_pattern.get()), nullptr);

  // A pattern has a default transform, the identity matrix
  EXPECT_EQ(test_pattern->transform, math::IDENTITY_MATRIX);

  // A pattern's transform may be assigned
  test_pattern->transform = math::translation(1, 2, 3);
  EXPECT_EQ(test_pattern->transform, math::translation(1, 2, 3));
}

TEST(PatternTest, ObjectPatternAt) {

  // A pattern with an object transformation
  auto object = std::make_shared<geo::Sphere>();
  object->transform = math::scaling(2, 2, 2);
  auto pattern = std::make_shared<pattern::TestPattern>();
  object->material.pattern = pattern;
  auto c = object->pattern_at(math::Point(2, 3, 4));
  EXPECT_EQ(c, color::Color(1, 1.5, 2));

  // A pattern with a pattern transformation
  object = std::make_shared<geo::Sphere>();
  pattern = std::make_shared<pattern::TestPattern>();
  pattern->transform = math::scaling(2, 2, 2);
  object->material.pattern = pattern;
  c = object->pattern_at(math::Point(2, 3, 4));
  EXPECT_EQ(c, color::Color(1, 1.5, 2));

  //  A pattern with both an object and a pattern transformation
  object = std::make_shared<geo::Sphere>();
  object->transform = math::scaling(2, 2, 2);
  pattern = std::make_shared<pattern::TestPattern>();
  pattern->transform = math::translation(0.5, 1, 1.5);
  object->material.pattern = pattern;
  c = object->pattern_at(math::Point(2.5, 3, 3.5));
  EXPECT_EQ(c, color::Color(0.75, 0.5, 0.25));
}

TEST(PatternTest, StripePatternEquality) {

  // A StripePattern derives from a Pattern
  auto stripe_pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  EXPECT_NE(dynamic_cast<pattern::Pattern*>(stripe_pattern.get()), nullptr);

  // Two StripePatterns with the same values are equal
  auto another_stripe_pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(*stripe_pattern, *another_stripe_pattern);

  // Two StripePatterns with the same values but different transforms are not equal
  another_stripe_pattern->transform = math::translation(1, 2, 3);
  EXPECT_NE(*stripe_pattern, *another_stripe_pattern);

  // Two StripePatterns with different values not equal
  another_stripe_pattern->transform = math::IDENTITY_MATRIX;
  another_stripe_pattern->a = color::BLACK;
  EXPECT_NE(*stripe_pattern, *another_stripe_pattern);

  //  StripePattern and a TestPattern are not equal
  auto test_pattern = std::make_shared<pattern::TestPattern>();
  //EXPECT_FALSE(*stripe_pattern == *test_pattern);
  EXPECT_NE(*stripe_pattern, *test_pattern);
}

TEST(PatternTest, StripePatternColors) {

  // Creating a stripe pattern
  auto pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(pattern->a, color::WHITE);
  EXPECT_EQ(pattern->b, color::BLACK);
}

TEST(PatternTest, StripePatternAt) {

  // A stripe pattern is constant in y
  auto pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 1, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 2, 0)), color::WHITE);

  // A stripe pattern is constant in z
  pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 1)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 2)), color::WHITE);

  // A stripe pattern alternates in x
  pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0.9, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(1, 0, 0)), color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(-0.1, 0, 0)), color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(-1, 0, 0)), color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(-1.1, 0, 0)), color::WHITE);
}

TEST(PatternTest, MaterialPattern){

  // lighting with a pattern applied on a sphere
  auto m = material::Material();
  m.pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  m.ambient = 1;
  m.diffuse = 0;
  m.specular = 0;
  auto s = std::make_shared<geo::Sphere>();  
  s->material = m;
  
  auto eye_v = math::Vector(0, 0, -1);
  auto normal_v = math::Vector(0, 0, -1);
  auto li = light::PointLight(math::Point(0, 0, -10), color::Color(1, 1, 1));

  auto c1 = lighting(s, li, math::Point(0.9, 0, 0), eye_v, normal_v, false);
  auto c2 = lighting(s, li, math::Point(1.1, 0, 0), eye_v, normal_v, false);
  EXPECT_EQ(c1, color::WHITE);
  EXPECT_EQ(c2, color::BLACK);
}

TEST(PatternTest, StripePatternAtWithTransformations) {

  // Stripes with an object transformation
  auto object = std::make_shared<geo::Sphere>();
  object->transform = math::scaling(2, 2, 2);
  auto pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  object->material.pattern = pattern;
  auto c = object->pattern_at(math::Point(1.5, 0, 0));
  EXPECT_EQ(c, color::WHITE);

  // Stripes with a pattern transformation
  object = std::make_shared<geo::Sphere>();
  pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  pattern->transform = math::scaling(2, 2, 2);
  object->material.pattern = pattern;
  c = object->pattern_at(math::Point(1.5, 0, 0));
  EXPECT_EQ(c, color::WHITE);

  // Stripes with both an object and a pattern transformation
  object = std::make_shared<geo::Sphere>();
  object->transform = math::scaling(2, 2, 2);
  pattern = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  pattern->transform = math::translation(0.5, 2, 2);
  object->material.pattern = pattern;
  c = object->pattern_at(math::Point(2.5, 0, 0));
  EXPECT_EQ(c, color::WHITE);
}

