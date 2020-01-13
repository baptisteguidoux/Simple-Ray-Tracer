/*! \file world.hpp
 */

#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory>
#include <optional>

#include "geo.hpp"
#include "light.hpp"
#include "ray.hpp"
#include "color.hpp"


/*! \namespace world
 */
namespace world {

  /*! \class World
   */
  class World {
  public:
    
    std::vector<std::shared_ptr<geo::Shape>> objects;
    std::optional<light::PointLight> light;

    /*! \fn bool contains_object(geo::Shape* object) const
     *  \brief Check if a geometry can be found in the World
     *  \param object shared_ptr to the object to check if it is present
     *  \return true if the given object is contained in the World
     */    
    bool contains_object(geo::Shape* object) const;

    /*! \fn geo::Intersections intersects(const ray::Ray& ry) const
     *  \brief Check if a Ray intersects any object in the World
     *  \param ry the Ray crossing the World
     *  \return every Intersections
     */    
    geo::Intersections intersects(const ray::Ray& ry) const;

    /*! \fn bool is_shadowed(const math::Tuple& light_position, const math::Tuple& point) const
     *  \brief Cast a shadow ray between the Intersection and the light position to find if the given point is in shadow
     *  \param point the point to pcheck
     *  \param light_position the position of the light
     *  \return true if the point is in shadow
     */      
    bool is_shadowed(const math::Tuple& light_position, const math::Tuple& point) const;

    /*! \fn float intensity_at(const light::PointLight light, const math::Tuple& point) const
     *  \brief Evlauates the light intensity at a given point
     *  \param light the Light for whch the intensity is calculated
     *  \param point the Point to look intensity
     *  \return a value between 0 and 1
     */    
    float intensity_at(const light::PointLight light, const math::Tuple& point) const;

    /*! \fn color::Color shade_hit(const geo::Computations& comps, const int remaining) const
     *  \brief Find the color at the point, in the World
     *  \param comps the Conputations containing the point
     *  \param remaining to avoid infinite recursion
     *  \return the Color at the point
     */
    color::Color shade_hit(const geo::Computations& comps, const int remaining = 5) const;

    /*! \fn color::Color color_at(const ray::Ray& ry, const int remaining) const
     *  \brief Wrapper func: check if the Ray intersects, prepare the Computations and calls shade_hit
     *  \param ry cast Ray
     *  \param remaining to avoid infinite recursion
     *  \return the Color for the Ray
     */    
    color::Color color_at(const ray::Ray& ry, const int remaining = 5) const;

    /*! \fn color::Color reflected_color(const geo::Computations& comps, const int remaining = 5) const
     *  \brief Computes the amount of reflected color
     *  \params comps Computations
     *  \param remaining  to avoid infinite recursion
     *  \return the reflected Color
     */    
    color::Color reflected_color(const geo::Computations& comps, const int remaining = 5) const;

    /*! \fn color::Color refracted_color(const geo::Computations& comps, const int remaining = 5) cons
     *  \brief Computes the amount of refracted Color
     *  \params comps Computations
     *  \param remaining to avoid infinite recursion
     *  \return the refracted Color
     */
    color::Color refracted_color(const geo::Computations& comps, const int remaining = 5) const;

  };

  /*! \fn World build_default_world()
   *  \brief default World has a light, and two spheres
   *  \return the default World
   */
  World build_default_world();
}

#endif

