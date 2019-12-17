
#ifndef PATTERN_H
#define PATTERN_H

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

}

#endif 

