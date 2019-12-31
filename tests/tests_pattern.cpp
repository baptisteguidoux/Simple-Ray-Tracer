
#include <memory>
#include <cmath>

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

  auto c1 = lighting(s.get(), li, math::Point(0.9, 0, 0), eye_v, normal_v, false);
  auto c2 = lighting(s.get(), li, math::Point(1.1, 0, 0), eye_v, normal_v, false);
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

TEST(PatternTest, GradientPattern) {

  // A GradientPattern derives from class Pattern
  auto pattern = std::make_shared<pattern::GradientPattern>(color::WHITE, color::BLACK);
  EXPECT_NE(static_cast<pattern::Pattern*>(pattern.get()), nullptr);

  // A GradientPattern linearly interpolates between colors
  pattern = std::make_shared<pattern::GradientPattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(0.25, 0, 0)), color::Color(0.75, 0.75, 0.75));
  EXPECT_EQ(pattern->pattern_at(math::Point(0.5, 0, 0)), color::Color(0.5, 0.5, 0.5));
  EXPECT_EQ(pattern->pattern_at(math::Point(0.75, 0, 0)), color::Color(0.25, 0.25, 0.25));

  // GradientPattern equality
  auto pattern2 = std::make_shared<pattern::GradientPattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(*pattern, *pattern2);
  pattern2->end_color = color::Color(1, 0, 0);
  EXPECT_NE(*pattern, *pattern2);
}

TEST(PatternTest, RingPattern) {

  // A RingPattern derives from class Pattern
  auto pattern = std::make_shared<pattern::RingPattern>(color::WHITE, color::BLACK);
  EXPECT_NE(dynamic_cast<pattern::Pattern*>(pattern.get()), nullptr);

  // A ring should extend in both x and z
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(1, 0, 0)), color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 1)), color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(sqrt(2) / 2 + 0.001, 0, sqrt(2) / 2 + 0.001)), color::BLACK);

  //RingPattern equality
  auto pattern2 = std::make_shared<pattern::RingPattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(*pattern, *pattern2);
  pattern2->a = color::Color(1, 0, 0);
  EXPECT_NE(*pattern, *pattern2);
}

TEST(PatternTest, CheckerPattern) {

  // A CheckerPattern derives from class Pattern
  auto pattern = std::make_shared<pattern::CheckerPattern>(color::WHITE, color::BLACK);
  EXPECT_NE(dynamic_cast<pattern::Pattern*>(pattern.get()), nullptr);

  // Checkers should repeat in x
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0.99, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(1.01, 0, 0)), color::BLACK);

  // Checkers should repeat in y
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0.99, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 1.01, 0)), color::BLACK);

  // Checkers should repeat in z
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0.99)), color::WHITE);
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 1.01)), color::BLACK);  
}

TEST(PatternTest, RadialGradientPattern) {

  // A RadialGradientPattern derives from class Pattern
  auto pattern = std::make_shared<pattern::RadialGradientPattern>(color::WHITE, color::BLACK);
  EXPECT_NE(static_cast<pattern::Pattern*>(pattern.get()), nullptr);

  // A RadialGradientPattern linearly interpolates between colors
  pattern = std::make_shared<pattern::RadialGradientPattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(pattern->pattern_at(math::Point(0.25, 0, 0)), color::Color(0.96922, 0.96922, 0.96922));
  EXPECT_EQ(pattern->pattern_at(math::Point(0.5, 0, 0)), color::Color(0.88196, 0.88196, 0.88196));
  EXPECT_EQ(pattern->pattern_at(math::Point(0.75, 0, 0)), color::Color(0.75, 0.75, 0.75));

  // RadialGradientPattern equality
  auto pattern2 = std::make_shared<pattern::RadialGradientPattern>(color::WHITE, color::BLACK);
  EXPECT_EQ(*pattern, *pattern2);
  pattern2->end_color = color::Color(1, 0, 0);
  EXPECT_NE(*pattern, *pattern2);
}

TEST(PatternTest, NestedPattern) {

  // A NestedPattern derives from class Pattern
  auto sub_pattern1 = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  auto sub_pattern2 = std::make_shared<pattern::StripePattern>(color::Color(1, 0, 0), color::Color(0, 1, 0));
  auto pattern = std::make_shared<pattern::NestedPattern>(sub_pattern1.get(), sub_pattern2.get());
  EXPECT_NE(static_cast<pattern::Pattern*>(pattern.get()), nullptr);

  // A NestedPattern alternates between two patterns like a checker
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)), pattern->sub_pattern1->pattern_at(math::Point(0, 0, 0)));
  EXPECT_EQ(pattern->pattern_at(math::Point(0.9, 0, 0.9)), pattern->sub_pattern1->pattern_at(math::Point(0, 0, 0)));
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0.9, 0)), pattern->sub_pattern1->pattern_at(math::Point(0, 0, 0)));
  EXPECT_EQ(pattern->pattern_at(math::Point(1.1, 0, 0)), pattern->sub_pattern2->pattern_at(math::Point(1.1, 0, 0)));
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 1.1)), pattern->sub_pattern2->pattern_at(math::Point(0, 0, 1.1)));
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 1.1, 0)), pattern->sub_pattern2->pattern_at(math::Point(0, 1.1, 0)));

  // NestedPattern equality
  auto pattern2 = std::make_shared<pattern::NestedPattern>(sub_pattern1.get(), sub_pattern2.get());
  EXPECT_EQ(*pattern, *pattern2);
  pattern2->sub_pattern1 = std::make_shared<pattern::RadialGradientPattern>(color::WHITE, color::BLACK);
  EXPECT_NE(*pattern, *pattern2);				       
}

TEST(PatternTest, BlendedPattern) {

  // A BlendedPattern derives from class Pattern
  auto sub_pattern1 = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  auto sub_pattern2 = std::make_shared<pattern::RingPattern>(color::Color(1, 0, 0), color::Color(0, 1, 0));
  auto pattern = std::make_shared<pattern::BlendedPattern>(sub_pattern1.get(), sub_pattern2.get());
  EXPECT_NE(static_cast<pattern::Pattern*>(pattern.get()), nullptr);

  // A BlendedPattern adds two Pattern
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 0)),
	    pattern->sub_pattern1->pattern_at(math::Point(0, 0, 0)) + pattern->sub_pattern2->pattern_at(math::Point(0, 0, 0)));
  EXPECT_EQ(pattern->pattern_at(math::Point(1, 0, 0)),
	    pattern->sub_pattern1->pattern_at(math::Point(1, 0, 0)) + pattern->sub_pattern2->pattern_at(math::Point(1, 0, 0)));
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 1, 0)),
	    pattern->sub_pattern1->pattern_at(math::Point(0, 1, 0)) + pattern->sub_pattern2->pattern_at(math::Point(0, 1, 0)));
  EXPECT_EQ(pattern->pattern_at(math::Point(0, 0, 1)),
	    pattern->sub_pattern1->pattern_at(math::Point(0, 0, 1)) + pattern->sub_pattern2->pattern_at(math::Point(0, 0, 1)));    

  // BlendedPattern equality
  auto pattern2 = std::make_shared<pattern::BlendedPattern>(sub_pattern1.get(), sub_pattern2.get());
  EXPECT_EQ(*pattern, *pattern2);
  pattern2->sub_pattern1 = std::make_shared<pattern::RadialGradientPattern>(color::WHITE, color::BLACK);
  EXPECT_NE(*pattern, *pattern2);  
}

TEST(PatternTest, PerturbedPattern) {

  // A PerturbedPattern derives from class Pattern
  auto sub_pattern = std::make_shared<pattern::TestPattern>();
  auto pattern = std::make_shared<pattern::PerturbedPattern>(sub_pattern.get());
  EXPECT_NE(static_cast<pattern::Pattern*>(pattern.get()), nullptr);

  // A PerturbedPattern just jitter the given point before passing to a "true" Pattern
  // Since TestPattern.local_pattern_at returns a Color(r,g,b) for a Point(x, y, z) where each component is equal to its counterpart
  // simply test returned Color values are different from Point
  auto point = math::Point(0.1, 0.35, 0.82);
  auto col = pattern->pattern_at(point);
  EXPECT_FALSE(math::almost_equal(point.x, col.red));
  EXPECT_FALSE(math::almost_equal(point.y, col.green));
  EXPECT_FALSE(math::almost_equal(point.z, col.blue));

  // PerturbedPattern equality
  auto sub_pattern2 = std::make_shared<pattern::StripePattern>(color::WHITE, color::BLACK);
  auto pattern2 = std::make_shared<pattern::PerturbedPattern>(sub_pattern2.get());
  auto pattern3 = std::make_shared<pattern::PerturbedPattern>(sub_pattern2.get());
  EXPECT_EQ(*pattern2, *pattern3);
  pattern3->sub_pattern = std::make_shared<pattern::RadialGradientPattern>(color::WHITE, color::BLACK);
  EXPECT_NE(*pattern, *pattern2);
}

