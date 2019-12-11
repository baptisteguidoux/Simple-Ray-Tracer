/* \file light.hpp
 */

#ifndef LIGHT_H
#define LIGHT_H

#include <memory>

#include "geo.hpp"
#include "tuple.hpp"
#include "color.hpp"


/* \namespace light
 */
namespace light {

  /* \class PointLight
   */
  class PointLight {
  public:

    math::Tuple position; 
    color::Color intensity;

    /* PointLight constructor
     * \param pos PointLight's position
     * \param int_ PointLight's color & intensity
     */
    PointLight(const math::Tuple& pos, const color::Color& int_);
    
  };

  /* \fn bool operator==(const PointLight& first, const PointLight& second)
   * \brief Compares the equality of two PointLight
   * \param first a PointLight
   * \param second another PointLight
   * \return true if the two PointLight are equal, false otherwise
   */  
  bool operator==(const PointLight& first, const PointLight& second);

  /* \fn bool operator==(const PointLight& first, const PointLight& second)
   * \brief Compares the unequality of two PointLight
   * \param first a PointLight
   * \param second another PointLight
   * \return true if the two PointLight are unequal, false otherwise
   */    
  bool operator!=(const PointLight& first, const PointLight& second);

  /* \fn color::Color lighting(std::shared_ptr<geo::Shape> object, const PointLight light, const math::Tuple& position, const math::Tuple& eye_vector, const math::Tuple& normal_vector, const bool in_shadow)
   * Calculate the Color of the Shape's Material at the given position
   * \param object lit Shape
   * \param light the scene's light
   * \param position the position of the point on the object
   * \param eye_vector vector surface --> eye
   * \param normal_vector object's surface normal
   * \param in_shadow, true if the point is in the shadow
   * \return the Color at this point
   */  
  color::Color lighting(std::shared_ptr<geo::Shape> object, const PointLight light, const math::Tuple& position,
			const math::Tuple& eye_vector, const math::Tuple& normal_vector, const bool in_shadow);

}

#endif

