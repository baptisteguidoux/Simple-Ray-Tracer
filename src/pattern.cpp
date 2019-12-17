#include <cmath>

#include "pattern.hpp"
#include "color.hpp"
#include "tuple.hpp"


namespace pattern {

  // Necessary to provide a definition for virtual destructors
  Pattern::~Pattern() {};

  color::Color Pattern::pattern_at(const math::Tuple& object_point) const {

    auto pattern_point = math::inverse(transform) * object_point;

    return local_pattern_at(pattern_point);
  }

  bool Pattern::operator==(const Pattern& other) const {

    return typeid(*this) == typeid(other) && this->transform == other.transform && this->local_equality_predicate(other);
  }

  bool Pattern::operator!=(const Pattern& other) const {
    
    return !(*this == other);
  }

  color::Color TestPattern::local_pattern_at(const math::Tuple& pattern_point) const {
    
    return color::Color(pattern_point.x, pattern_point.y, pattern_point.z);
  }

  TestPattern::~TestPattern() {};

  bool TestPattern::local_equality_predicate(const Pattern& other) const {
    // No own member variables
    return true;
  }
  
  StripePattern::StripePattern(const color::Color& a_, const color::Color& b_) : a{a_}, b{b_} {};
  
  StripePattern::~StripePattern() {};
  
  color::Color StripePattern::local_pattern_at(const math::Tuple& pattern_point) const {

    if (static_cast<int>(floor(pattern_point.x)) % 2 == 0)
      return a;

    return b;
  }

  bool StripePattern::local_equality_predicate(const Pattern& other) const {

    // Required to actually compare two StripePattern, not a Pattern and a StripePatterb
    const auto& cast_other = dynamic_cast<const StripePattern&>(other);
    
    return (this->a == cast_other.a) && (this->b == cast_other.b);
  } 

  GradientPattern::GradientPattern(const color::Color& start, const color::Color& end) : start_color{start}, end_color{end} {}

  GradientPattern::~GradientPattern() {};

  color::Color GradientPattern::local_pattern_at(const math::Tuple& pattern_point) const {

    auto color_distance = end_color - start_color;
    auto fraction = pattern_point.x - floor(pattern_point.x);
    
    return start_color + color_distance * fraction;
  }
  
  bool GradientPattern::local_equality_predicate(const Pattern& other) const {

    const auto& cast_other = static_cast<const GradientPattern&>(other);

    return (this->start_color == cast_other.start_color) && (this->end_color == cast_other.end_color);
  }

  RingPattern::RingPattern(const color::Color& a_, const color::Color& b_) : a {a_}, b {b_} {}

  RingPattern::~RingPattern() {};

  color::Color RingPattern::local_pattern_at(const math::Tuple& pattern_point) const {

    if (static_cast<int>(floor(sqrt(pattern_point.x * pattern_point.x + pattern_point.z * pattern_point.z))) % 2 == 0)
      return a;
    else
      return b;
  }

  bool RingPattern::local_equality_predicate(const Pattern& other) const {

    const auto& cast_other = static_cast<const RingPattern&>(other);
    
    return (this->a == cast_other.a) && (this->b == cast_other.b);
  }

  CheckerPattern::CheckerPattern(const color::Color& a_, const color::Color& b_) : a {a_}, b {b_} {}

  CheckerPattern::~CheckerPattern() {};

  color::Color CheckerPattern::local_pattern_at(const math::Tuple& pattern_point) const {

    if (static_cast<int>(floor(pattern_point.x) + floor(pattern_point.y) + floor(pattern_point.z)) % 2 == 0)
      return a;
    else
      return b;
  }

  bool CheckerPattern::local_equality_predicate(const Pattern& other) const {

    const auto& cast_other = static_cast<const CheckerPattern&>(other);
    
    return (this->a == cast_other.a) && (this->b == cast_other.b);
  }

  RadialGradientPattern::RadialGradientPattern(const color::Color& start, const color::Color& end) : start_color{start}, end_color{end} {}

  RadialGradientPattern::~RadialGradientPattern() {};

  color::Color RadialGradientPattern::local_pattern_at(const math::Tuple& pattern_point) const {

    auto color_distance = end_color - start_color;
    auto point_dist_from_orig = math::magnitude(pattern_point); // radial distance
    auto fraction = point_dist_from_orig - floor(point_dist_from_orig);
    
    return start_color + color_distance * fraction;
  }
  
  bool RadialGradientPattern::local_equality_predicate(const Pattern& other) const {

    const auto& cast_other = static_cast<const RadialGradientPattern&>(other);

    return (this->start_color == cast_other.start_color) && (this->end_color == cast_other.end_color);
  }

  NestedPattern::NestedPattern(const std::shared_ptr<Pattern>& sb1, const std::shared_ptr<Pattern>& sb2) : sub_pattern1 {sb1}, sub_pattern2 {sb2} {}

  NestedPattern::~NestedPattern() {};

  color::Color NestedPattern::local_pattern_at(const math::Tuple& pattern_point) const {
    
    if (static_cast<int>(floor(pattern_point.x) + floor(pattern_point.y) + floor(pattern_point.z)) % 2 == 0)
      return sub_pattern1->pattern_at(pattern_point);
    else
      return sub_pattern2->pattern_at(pattern_point);
  }

  bool NestedPattern::local_equality_predicate(const Pattern& other) const {

    const auto& cast_other = static_cast<const NestedPattern&>(other);

    return (*sub_pattern1 == *cast_other.sub_pattern1) && (*sub_pattern2 == *cast_other.sub_pattern2);
  }

  BlendedPattern::BlendedPattern(const std::shared_ptr<Pattern>& sb1, const std::shared_ptr<Pattern>& sb2) : sub_pattern1 {sb1}, sub_pattern2 {sb2} {}

  BlendedPattern::~BlendedPattern() {};

  color::Color BlendedPattern::local_pattern_at(const math::Tuple& pattern_point) const {
    
    return sub_pattern1->pattern_at(pattern_point) + sub_pattern2->pattern_at(pattern_point);
  }

  bool BlendedPattern::local_equality_predicate(const Pattern& other) const {

    const auto& cast_other = static_cast<const BlendedPattern&>(other);

    return (*sub_pattern1 == *cast_other.sub_pattern1) && (*sub_pattern2 == *cast_other.sub_pattern2);
  }
    
  PerturbedPattern::PerturbedPattern(const std::shared_ptr<Pattern>& sb) : sub_pattern {sb} {}

  PerturbedPattern::~PerturbedPattern() {};
  
  color::Color PerturbedPattern::local_pattern_at(const math::Tuple& pattern_point) const {

    // Value between -1 and 1
    const auto noise_value = noise.Evaluate(pattern_point.x, pattern_point.y, pattern_point.z);
    // Jitter between 0 and 20%
    const auto jitter = math::map(noise_value, -1, 1, 0.8, 1.2);
    
    return sub_pattern->local_pattern_at(pattern_point * jitter);
  }

  bool PerturbedPattern::local_equality_predicate(const Pattern& other) const {

    const auto& cast_other = static_cast<const PerturbedPattern&>(other);

    return (sub_pattern == cast_other.sub_pattern);
  }

}
