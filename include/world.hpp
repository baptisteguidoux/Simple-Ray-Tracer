/* \file world.hpp
 */

#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <memory>
#include <optional>

#include "geo.hpp"
#include "light.hpp"
#include "ray.hpp"


/* \namespace world
 */
namespace world {

  /* \class World
   */
  class World {
  public:
    
    std::vector<std::shared_ptr<geo::Shape>> objects;
    std::optional<light::PointLight> light;

    /* \fn bool contains_object(std::shared_ptr<geo::Shape> object) const
     * \brief Check if a geometry can be found in the World
     * \param object shared_ptr to the object to check if it is present
     * \return true if the given object is contained in the World
     */    
    bool contains_object(std::shared_ptr<geo::Shape> object) const;

    /* \fn geo::Intersections intersects(const ray::Ray& ry) const
     * \brief Check if a Ray intersects any object in the World
     * \param ry the Ray crossing the World
     * \return every Intersections
     */    
    geo::Intersections intersects(const ray::Ray& ry) const;

  };

  /* \fn World build_default_world()
   * default World has a light, and two spheres
   * \return the default World
   */
  World build_default_world();
}

#endif

