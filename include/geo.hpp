/* \file geo.hpp
 */

#ifndef GEO_H
#define GEO_H

#include <vector>
#include <optional>
#include <memory>

#include "ray.hpp"
#include "tuple.hpp"
#include "matrix.hpp"
#include "material.hpp"


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
    material::Material material = material::Material();

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
  
  /* \fn bool operator==(const Shape& first, const Shape& second)
   * \brief Compares the equality of two Shape
   * \param first a Shape
   * \param second another Shape
   * \return true if the two Shape are equal, false otherwise
   */  
  bool operator==(const Shape& first, const Shape& second);

  /* \fn bool operator!=(const Shape& first, const Shape& second)
   * \brief Compares the unequality of two Shape
   * \param first a Shape
   * \param second another Shape
   * \return true if the two Shape are not equal, false otherwise
   */   
  bool operator!=(const Shape& first, const Shape& second);

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

  class Plane : public Shape {
  public:

    ~Plane() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point) const override;
  };
  
  /* \fn math::Tuple reflect(const math::Tuple& vec, const math::Tuple& normal)
   * Reflects the vector around the normal
   * \param the vector to reflect
   * \parm the normal vector
   * \return a Tuple, the reflected vector
   */ 
  math::Tuple reflect(const math::Tuple& vec, const math::Tuple& normal);
  
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

    Intersection() = default;

    /* \fn Intersection& operator=(const Intersection& source)
     * \param source the Intersection to copied, the source
     * \return Intersection copy destination
     */       
    Intersection& operator=(const Intersection& source);

  };
  /* \fn std::optional<Intersection> hit(const Intersections& intersections)
   * Find the hit among all the Intersections, if there is any. The hit is the lowest nonegative intersection.
   * \param the Intersections to look at
   * \return an optional Intersection
   */
  std::optional<Intersection> hit(const Intersections& intersections);

  /* \fn bool operator==(const Intersection& first, const Intersection& second)
   * \brief Compares the equality of two Intersection
   * \param first an Intersection
   * \param second another Intersection
   * \return true if the two Intersection are equal, false otherwise
   */
  bool operator==(const Intersection& first, const Intersection& second);

  /* \fn bool operator!=(const Intersection& first, const Intersection& second)
   * \brief Compares the unequality of two Intersection
   * \param first an Intersection
   * \param second another Intersection
   * \return true if the two Intersection are not equal, false otherwise
   */  
  bool operator!=(const Intersection& first, const Intersection& second);

  
  /*! \struct Computations
   *  \brief Store some precomputed values relative to an intersection
   */
  struct Computations {

    double t;
    std::shared_ptr<geo::Shape> geometry;
    math::Tuple point;
    math::Tuple eye_vector;
    math::Tuple normal_vector;
    bool inside; /*!< true if the intersection occured into an object*/
    math::Point over_point = math::Point(0, 0, 0); /*!< Adjust the point slightly above the norrmal to prevent self-shadowing and acne due to float rounding errors*/
    
    /*! Computations' constuctor
     */
    Computations() = default;
  };  

  /*! \fn Computations prepare_computations(const inter::Intersection& ixs, const ray::Ray r)
   *  \param ixs an Intersection
   *  \param r a Ray
   *  \return a Computations
   */
  Computations prepare_computations(const Intersection& ixs, const ray::Ray r);
  //Computations prepare_computations(const Intersection& ix, const ray::Ray r, const Intersections& ixs);
  
}

#endif
  
