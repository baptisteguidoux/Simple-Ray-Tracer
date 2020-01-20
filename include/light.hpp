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

  /*! \struct SequenceGenerator
   *  \brief useful to sample a random subset of points on the AreaLight
   */
  struct SequenceGenerator {
    std::vector<float> m_seq;
    size_t index = 0;

    SequenceGenerator(std::initializer_list<float> float_seq);

    SequenceGenerator() = default;

    float next();
    
  };

  /*! \class Light
   *  \brief Base class for all lights
   */
  class Light {
  public:
    math::Tuple m_position;
    color::Color m_intensity;

    Light(const math::Tuple& pos, const color::Color& int_);

    virtual ~Light() = 0;

    /*! \fn float intensity_at(const math::Tuple& point, const world::World& world)
     *  \brief Evaluates the light intensity at a given point
     *  \param point the Point to look intensity
     *  \param world the World where the Point is
     *  \return a float value between 0 and 1
     */
    virtual float intensity_at(const math::Tuple& point, const world::World& wrld) = 0;
    
    /*! \fn color::Color lighting(geo::Shape* object, const math::Tuple& position, const math::Tuple& eye_vector, 
                                  const math::Tuple& normal_vector, const  float intensity)
     *  \brief Calculates the Color of the Shape's Material at the given position
     *  \param object lit Shape
     *  \param position the position of the point on the object
     *  \param eye_vector vector surface --> eye
     *  \param normal_vector object's surface normal
     *  \param intensity how much light is present
     *  \return the Color at this point
     */     
    virtual color::Color lighting(geo::Shape* object, const math::Tuple& position, const math::Tuple& eye_vector, 
				  const math::Tuple& normal_vector, const float intensity) = 0;
  };

  /*! \class PointLight
   */
  class PointLight : public Light {
  public:

    /* PointLight constructor
     * \param pos PointLight's position
     * \param int_ PointLight's color & intensity
     */
    PointLight(const math::Tuple& pos, const color::Color& int_);

    /*! PointLight constructor
     */    
    PointLight();

    ~PointLight();

    /*! \fn float intensity_at(const light::PointLight light, const math::Tuple& point) const
     *  \brief Evaluates the light intensity at a given point
     *  \param point the Point to look intensity
     *  \param world the World where the Point is
     *  \return a value between 0 and 1
     */    
    float intensity_at(const math::Tuple& point, const world::World& wrld) override;
 
    color::Color lighting(geo::Shape* object, const math::Tuple& position, const math::Tuple& eye_vector, 
			  const math::Tuple& normal_vector, const float intensity) override;
    
  };

  /*! \class AreaLight
   *  \brief a flat, rectangular light source
   */
  class AreaLight : public Light {
  public:
    
    math::Tuple corner; /// position of one corner of the light  source
    math::Tuple uvec; /// u dimension of a single cell
    uint usteps; // how many points are sampled along the u edge
    math::Tuple vvec;
    uint vsteps;
    uint samples; /// number of cells (samples) in the area light, usteps * vsteps
    SequenceGenerator jitter_by; /// to randomly select the point in each cell

    AreaLight(const math::Tuple& corner_, const math::Tuple& uvec_, const uint usteps_,
	      const math::Tuple& vvec_, const uint vsteps_, const color::Color& intensity_);

    ~AreaLight();
    
    /*! \fn math::Tuple point_at(const uint u, const uint v)
     *  \brief get the point in the middle of the cell at the given coordinates
     *  \param u u coordinate
     *  \param v v coordinate
     *  \return the Point at the middle of the cell
     */
    math::Tuple point_at(const uint u, const uint v);

    /*! \fn float intensity_at(const math::Tuple& point, const world::World& world)
     *  \brief Evaluates the light intensity at a given point
     *  \param point the Point to look intensity
     *  \param world the World where the Point is
     *  \return a float value between 0 and 1
     */
    float intensity_at(const math::Tuple& point, const world::World& wrld) override;

    color::Color lighting(geo::Shape* object, const math::Tuple& position, const math::Tuple& eye_vector, 
			  const math::Tuple& normal_vector, const float intensity) override;     
  };

  /*! \fn bool operator==(const PointLight& first, const PointLight& second)
   *  \brief Compares the equality of two PointLight
   *  \param first a PointLight
   *  \param second another PointLight
   *  \return true if the two PointLight are equal, false otherwise
   */  
  bool operator==(const Light& first, const Light& second);

  /*! \fn bool operator==(const PointLight& first, const PointLight& second)
   *  \brief Compares the unequality of two PointLight
   *  \param first a PointLight
   *  \param second another PointLight
   *  \return true if the two PointLight are unequal, false otherwise
   */    
  bool operator!=(const Light& first, const Light& second);

}

#endif

