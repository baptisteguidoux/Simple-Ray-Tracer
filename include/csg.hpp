/*! \file csg.hpp
 */

#ifndef CSG_H
#define CSG_H

#include <memory>
#include <string>

#include "geo.hpp"


namespace geo {

  /*! \class CSG
   *  \brief A Shape that is the result of set operations on two Shapes
   */
  class CSG : public Shape {
  public:

    std::string operation; /// the set operation that created this CSG Shape
    std::shared_ptr<geo::Shape> left;
    std::shared_ptr<geo::Shape> right;

    CSG(const std::string& set_op, const std::shared_ptr<geo::Shape> le, const std::shared_ptr<geo::Shape> ri);
    
    ~CSG() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    /*! \fn  BoundingBox get_bounds() const 
     *  \brief Convert the BoundingBox of its children into group space
     *  \return CSG's BoundingBox
     */
    BoundingBox get_bounds() const override;

  };  

  /*! \fn std::shared_ptr<geo::CSG> operator|(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second)
   *  \brief Union of 2 Shapes
   *  \param first a Shape
   *  \param second another Shape
   *  \return a CSG Shape, product of the union of first and second
   */
  std::shared_ptr<geo::CSG> operator|(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second);
  
}

#endif

