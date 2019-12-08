/* \file geo.hpp
 */

#ifndef GEO_H
#define GEO_H

#include <vector>

#include "ray.hpp"
#include "tuple.hpp"
#include "matrix.hpp"


/* \namespace geo
 */
namespace geo {

  // Forward declaration
  struct Intersection;
  typedef std::vector<Intersection> Intersections;


  /* \class Shape
   * Base class, with some virtual functions
   */
  class Shape {
  public:

    /* virtual destructor
     */
    virtual ~Shape() = 0;

    math::Matrix transform = math::IDENTITY_MATRIX; /*!< default transform*/

    /* \fn Intersections intersects(const ray::Ray& ray)
     * Computes the Intersections of the Ray and Shape, calls local_intersects
     * \param ray the Ray to intersect with
     * \return Intersections
     */
    Intersections intersects(const ray::Ray& ray);

    /* \fn virtual Intersections local_intersects(const ray::Ray& local_ray) = 0
     * Shape dereived classes must implement this function to locally compute the Intersections of the Ray and Shape
     * \param ray the Ray to intersect with
     * \return Intersections
     */
    virtual Intersections local_intersects(const ray::Ray& local_ray) = 0;

    /* \fn math::Tuple normal_at(const math::Tuple& world_point)
     * Calculates the normal on the Shape at the given point (in world space)
     * \param world_point
     * \return a Tuple, the normal Vector (wolrd space)
     */    
    math::Tuple normal_at(const math::Tuple& world_point);

    /* \fn virtual math::Tuple local_normal_at(const math::Tuple& local_point) const = 0
     * Shape dereived classes must implement this function to locally calculate the normal on the Shape
     * \param local_point the point in local space
     * \return a Tuple, the normal Vector (local space)
     */     
    virtual math::Tuple local_normal_at(const math::Tuple& local_point) const = 0;
    
  };

  /* \class TestShape
   */  
  class TestShape : public Shape {
  public:

    ~TestShape() override;
   
    ray::Ray saved_ray = ray::Ray(math::Point(0, 0, 0), math::Vector(0, 0, 0));

    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point) const override;

  };

  class Sphere : public Shape {
  public:

    ~Sphere() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point) const override;
  };
  
  
  /* \struct Intersection
   */
  struct Intersection {

    float t;
    std::shared_ptr<geo::Shape> geometry;

    /* Intersection constructor
     * \param t_ the t value of the intersection, "when" the ray intersects the geo
     * \param geo the object that's been intersected
     */   
    Intersection(const float t_, std::shared_ptr<geo::Shape> geo);
    
  };

}

#endif
  
