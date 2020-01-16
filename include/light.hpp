/*! \file light.hpp
 */

#ifndef LIGHT_H
#define LIGHT_H

#include <memory>

#include "geo.hpp"
#include "tuple.hpp"
#include "color.hpp"


// Forward declaration
namespace world {
  class World;
}

/*! \namespace light
 */
namespace light {

  /*! \class PointLight
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

    /*! PointLight constructor
     */    
    PointLight();

    /*! \fn float intensity_at(const light::PointLight light, const math::Tuple& point) const
     *  \brief Evaluates the light intensity at a given point
     *  \param point the Point to look intensity
     *  \param world the World where the Point is
     *  \return a value between 0 and 1
     */    
    float intensity_at(const math::Tuple& point, const world::World& wrld) const;
    
  };

  /*! \class AreaLight
   *  \brief a flat, rectangular light source
   */
  class AreaLight {
  public:
    
    math::Tuple corner; /// position of one corner of the light  source
    math::Tuple uvec; /// u dimension of a single cell
    uint usteps; // how many points are sampled along the u edge
    math::Tuple vvec;
    uint vsteps;
    uint samples; /// number of cells (samples) in the area light, usteps * vsteps
    math::Tuple position; /// center of the AreaLight
    color::Color intensity;

    AreaLight(const math::Tuple& corner_, const math::Tuple& uvec_, const uint usteps_,
	      const math::Tuple& vvec_, const uint vsteps_, const color::Color& intensity_);

    /*! \fn math::Tuple point_at(const uint u, const uint v)
     *  \brief get the point in the middle of the cell at the given coordinates
     *  \param u u coordinate
     *  \param v v coordinate
     *  \return the Point at the middle of the cell
     */
    math::Tuple point_at(const uint u, const uint v) const;

    /*! \fn float intensity_at(const math::Tuple& point, const world::World& world)
     *  \brief Evaluates the light intensity at a given point
     *  \param point the Point to look intensity
     *  \param world the World where the Point is
     *  \return a float value between 0 and 1
     */
    float intensity_at(const math::Tuple& point, const world::World& wrld) const;
  };

  /*! \fn bool operator==(const PointLight& first, const PointLight& second)
   *  \brief Compares the equality of two PointLight
   *  \param first a PointLight
   *  \param second another PointLight
   *  \return true if the two PointLight are equal, false otherwise
   */  
  bool operator==(const PointLight& first, const PointLight& second);

  /*! \fn bool operator==(const PointLight& first, const PointLight& second)
   *  \brief Compares the unequality of two PointLight
   *  \param first a PointLight
   *  \param second another PointLight
   *  \return true if the two PointLight are unequal, false otherwise
   */    
  bool operator!=(const PointLight& first, const PointLight& second);

  /*! \fn color::Color lighting(geo::Shape* object, const PointLight light, const math::Tuple& position, const math::Tuple& eye_vector, const math::Tuple& normal_vector, const  float intensity)
   *  \brief Calculates the Color of the Shape's Material at the given position
   *  \param object lit Shape
   *  \param light the scene's light
   *  \param position the position of the point on the object
   *  \param eye_vector vector surface --> eye
   *  \param normal_vector object's surface normal
   *  \param intensity how much light is present
   *  \return the Color at this point
   */  
  color::Color lighting(geo::Shape* object, const PointLight light, const math::Tuple& position,
			const math::Tuple& eye_vector, const math::Tuple& normal_vector, const float intensity);

}

#endif

