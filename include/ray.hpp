/* \file ray.hpp
 */


#ifndef RAY_H
#define RAY_H

#include <memory>

#include "tuple.hpp"
#include "matrix.hpp"


/* \namespace ray
 */
namespace ray {

  /* \struct Ray
   */
  struct Ray{

    math::Tuple origin;
    math::Tuple direction;

    /* Ray constructor
     * \param ori Origin of the Ray
     * \param dir Direction of the Ray
     */
    Ray(const math::Tuple& ori, const math::Tuple& dir);

    /* \fn math::Point position(const float time) const
     * Find the position of the Ray after the given amount of time
     * \param time amount of time
     * \return a Point, the position of the Ray
     */
    math::Tuple position(const float time) const;

    /* \fn Ray transform(const math::Matrix& matrix) const
     * Constructs a new Ray by applying the transformation matrix to its origin and direction
     * \param matrix The transformation Matrix to apply
     * \return a new, transformed, Ray
     */
    Ray transform(const math::Matrix& matrix) const;
    
  };

}

#endif

