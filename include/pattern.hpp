
#ifndef PATTERN_H
#define PATTERN_H

#include <memory>

#include "matrix.hpp"
#include "color.hpp"


/* \namespace pattern
 */
namespace pattern {


  /* \class Pattern
   */  
  class Pattern {
  public:

    math::Matrix transform = math::IDENTITY_MATRIX;

    /* virtual destructor
     */
    virtual ~Pattern() = 0;    

    /* \fn color::Color pattern_at(const math::Tuple& object_point) const;
     * \brief Look for the Color of the Pattern at the given coordinates (in object space)
     * \param object_point The coordinates in object space
     * \return the Color at this Point
     */
    color::Color pattern_at(const math::Tuple& object_point) const;

    /* \fn virtual color::Color local_pattern_at(const math::Tuple& pattern_point) const = 0
     * \brief Look for the Color of the Pattern at the given coordinates (in pattern space)// pure virtual function to be implemented by derived classes
     * \param pattern_point the Point in Pattern coordinates
     * \return the Color at this Point
     */    
    virtual color::Color local_pattern_at(const math::Tuple& pattern_point) const = 0;

    /* \fn bool operator==(const Pattern& other) const
     * \brief Pattern equality predicate function
     * \param other another Pattern
     * \return true if the two Pattern are identical
     */
    bool operator==(const Pattern& other) const;

    /* \fn bool operator!=(const Pattern& other) const
     * \brief Pattern equality predicate function
     * \param other another Pattern
     * \return true if the two Pattern are different
     */    
    bool operator!=(const Pattern& other) const;

    /* \fn virtual bool local_equality_predicate(const Pattern& other) const = 0
     * \brief equality comparator function delegates to this one in the derived classes
     * \param other another Pattern
     * \return true if the two Pattern are identical, false otherwise
     */
    virtual bool local_equality_predicate(const Pattern& other) const = 0;

  };

  class TestPattern : public Pattern {
  public:
    ~TestPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  class StripePattern : public Pattern {
  public:
    color::Color a;
    color::Color b;
    
    StripePattern(const color::Color& a_, const color::Color& b_);

    ~StripePattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  class GradientPattern : public Pattern {
  public:
    color::Color start_color;
    color::Color end_color;
    
    GradientPattern(const color::Color& start, const color::Color& end);

    ~GradientPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  class RingPattern : public Pattern {
  public:
    color::Color a;
    color::Color b;
    
    RingPattern(const color::Color& a_, const color::Color& b_);

    ~RingPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };
  
  class CheckerPattern : public Pattern {
  public:
    color::Color a;
    color::Color b;
    
    CheckerPattern(const color::Color& a_, const color::Color& b_);

    ~CheckerPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  class RadialGradientPattern : public Pattern {
  public:
    color::Color start_color;
    color::Color end_color;
    
    RadialGradientPattern(const color::Color& start, const color::Color& end);

    ~RadialGradientPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  class NestedPattern : public Pattern {
  public:
    std::shared_ptr<Pattern> sub_pattern1;
    std::shared_ptr<Pattern> sub_pattern2;
    
    NestedPattern(const std::shared_ptr<Pattern>& sb1, const std::shared_ptr<Pattern>& sb2);

    ~NestedPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  class BlendedPattern : public Pattern {
  public:
    std::shared_ptr<Pattern> sub_pattern1;
    std::shared_ptr<Pattern> sub_pattern2;
    
    BlendedPattern(const std::shared_ptr<Pattern>& sb1, const std::shared_ptr<Pattern>& sb2);

    ~BlendedPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };  
  
}

#endif 

