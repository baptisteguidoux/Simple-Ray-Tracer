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

}

#endif

