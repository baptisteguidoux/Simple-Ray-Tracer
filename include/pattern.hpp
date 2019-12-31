/*! \file pattern.hpp
 */

#ifndef PATTERN_H
#define PATTERN_H

#include <memory>

#include "noise.hpp"

#include "matrix.hpp"
#include "color.hpp"


/*! \namespace pattern
 */
namespace pattern {


  /*! \class Pattern
   */  
  class Pattern : public std::enable_shared_from_this<Pattern> {
  public:

    math::Matrix transform = math::IDENTITY_MATRIX;

    /* virtual destructor
     */
    virtual ~Pattern() = 0;    

    /*! \fn color::Color pattern_at(const math::Tuple& object_point) const;
     *  \brief Look for the Color of the Pattern at the given coordinates (in object space)
     *  \param object_point The coordinates in object space
     *  \return the Color at this Point
     */
    color::Color pattern_at(const math::Tuple& object_point) const;

    /*! \fn virtual color::Color local_pattern_at(const math::Tuple& pattern_point) const = 0
     *  \brief Look for the Color of the Pattern at the given coordinates (in pattern space)// pure virtual function to be implemented by derived classes
     *  \param pattern_point the Point in Pattern coordinates
     *  \return the Color at this Point
     */    
    virtual color::Color local_pattern_at(const math::Tuple& pattern_point) const = 0;

    /*! \fn bool operator==(const Pattern& other) const
     * \brief Pattern equality predicate function
     * \param other another Pattern
     * \return true if the two Pattern are identical
     */
    bool operator==(const Pattern& other) const;

    /*! \fn bool operator!=(const Pattern& other) const
     *  \brief Pattern equality predicate function
     *  \param other another Pattern
     *  \return true if the two Pattern are different
     */    
    bool operator!=(const Pattern& other) const;

    /*! \fn virtual bool local_equality_predicate(const Pattern& other) const = 0
     *  \brief equality comparator function delegates to this one in the derived classes
     *  \param other another Pattern
     *  \return true if the two Pattern are identical, false otherwise
     */
    virtual bool local_equality_predicate(const Pattern& other) const = 0;

    /*! \fn std::shared_ptr<Shape> get_shared_ptr()
     *  \brief Get a shared_ptr which shares ownership of this Pattern
     *  \return a shared_ptr to *this
     */
    std::shared_ptr<Pattern> get_shared_ptr();

    /*! \fn std::weak_ptr<Shape> get_weak_ptr()
     *  \brief Get a weak_ptr which has a weak reference to this Pattern
     *  \return a weak_ptr to *this
     */    
    std::weak_ptr<Pattern> get_weak_ptr();

  };

  /*! \class TestPattern
   */   
  class TestPattern : public Pattern {
  public:
    ~TestPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class StripePattern
   */ 
  class StripePattern : public Pattern {
  public:
    color::Color a;
    color::Color b;
    
    StripePattern(const color::Color& a_, const color::Color& b_);

    ~StripePattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class GradientPattern
   *  \brief linearly interpolates between colors
   */   
  class GradientPattern : public Pattern {
  public:
    color::Color start_color;
    color::Color end_color;
    
    GradientPattern(const color::Color& start, const color::Color& end);

    ~GradientPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class RingPattern
   */  
  class RingPattern : public Pattern {
  public:
    color::Color a;
    color::Color b;
    
    RingPattern(const color::Color& a_, const color::Color& b_);

    ~RingPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class CheckerPattern
   */  
  class CheckerPattern : public Pattern {
  public:
    color::Color a;
    color::Color b;
    
    CheckerPattern(const color::Color& a_, const color::Color& b_);

    ~CheckerPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class RadialGradientPattern
   *  \brief radial interpolation between colors
   */  
  class RadialGradientPattern : public Pattern {
  public:
    color::Color start_color;
    color::Color end_color;
    
    RadialGradientPattern(const color::Color& start, const color::Color& end);

    ~RadialGradientPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class NestedPattern
   *  \brief alternates between two patterns, like a checker
   */  
  class NestedPattern : public Pattern {
  public:
    std::shared_ptr<Pattern> sub_pattern1;
    std::shared_ptr<Pattern> sub_pattern2;
    
    NestedPattern(Pattern* sb1, Pattern* sb2);

    ~NestedPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class BlendedPattern
   *  \brief adds two Pattern
   */  
  class BlendedPattern : public Pattern {
  public:
    std::shared_ptr<Pattern> sub_pattern1;
    std::shared_ptr<Pattern> sub_pattern2;
    
    BlendedPattern(Pattern* sb1, Pattern* sb2);

    ~BlendedPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };

  /*! \class PerturbedPattern
   *  \brief a sub Pattern is transformed using OpenSimplex noise
   */  
  class PerturbedPattern : public Pattern {
  public:
    std::shared_ptr<Pattern> sub_pattern;
    noise::OpenSimplexNoise noise;
    
    PerturbedPattern(Pattern* sb);

    ~PerturbedPattern() override;

    color::Color local_pattern_at(const math::Tuple& pattern_point) const override;

    bool local_equality_predicate(const Pattern& other) const override;
    
  };  
  
}

#endif 

