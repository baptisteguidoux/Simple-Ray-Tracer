/*! \file csg.hpp
 */

#ifndef CSG_H
#define CSG_H

#include <memory>
#include <string>

#include "geo.hpp"


namespace geo {

  enum class SetOperation : int {Union, Intersection, Difference};

  /*! \class CSG
   *  \brief A Shape that is the result of set operations on two Shapes
   */
  class CSG : public Shape {
  public:

    SetOperation operation;
    std::shared_ptr<geo::Shape> left;
    std::shared_ptr<geo::Shape> right;

    CSG(const SetOperation set_op, const std::shared_ptr<geo::Shape> le, const std::shared_ptr<geo::Shape> ri);
    
    ~CSG() override;
    
    Intersections local_intersects(const ray::Ray& local_ray) override;

    math::Tuple local_normal_at(const math::Tuple& local_point, const Intersection& ix) const override;

    bool local_equality_predicate(const Shape* shape) const override;

    /*! \fn  BoundingBox get_bounds() const 
     *  \brief Convert the BoundingBox of its children into group space
     *  \return CSG's BoundingBox
     */
    BoundingBox get_bounds() const override;

    /*! \fn bool includes(const Shape* shape) const
     *  \brief Recusrively check if the CSG contains the passed shape
     *  \param shape Shape to check if contained
     *  \return true of the Shape was found
     */
    bool includes(const Shape* shape) const;    

    /*! \fn Intersections filter_intersections(const Intersections& ixs)
     *  \brief Only keep the allowed Intersections
     *  \param ixs All Intersections occured with children
     *  \return The filtered Intersections
     */
    Intersections filter_intersections(const Intersections& ixs);

  };  

  /*! \fn std::shared_ptr<geo::CSG> operator|(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second)
   *  \brief Union of 2 Shapes
   *  \param first a Shape
   *  \param second another Shape
   *  \return a CSG Shape, product of the union of first and second
   */
  std::shared_ptr<geo::CSG> operator|(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second);

  /*! \fn std::shared_ptr<geo::CSG> operator&(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second)
   *  \brief Intersection of 2 Shapes
   *  \param first a Shape
   *  \param second another Shape
   *  \return a CSG Shape, product of the union of first and second
   */  
  std::shared_ptr<geo::CSG> operator&(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second);

  /*! \fn std::shared_ptr<geo::CSG> operator-(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second)
   *  \brief Difference of 2 Shapes
   *  \param first a Shape
   *  \param second another Shape
   *  \return a CSG Shape, product of the union of first and second
   */  
  std::shared_ptr<geo::CSG> operator-(const std::shared_ptr<geo::Shape> first, const std::shared_ptr<geo::Shape> second);  

  /*! \fn bool intersection_allowed(const SetOperation operation, const bool left_hit, const bool in_left, const bool in_right, const bool result)
   *  \brief Find whether an Intersection can occur with this kind of operation, in those parameters. Filter out the Intersections inside two objects
   *  \param operation a set operation: union, intersection or difference
   *  \param left_hit true if the left Shape is hit, false if the right Shape is hit
   *  \param in_left true if the hit occurs in the left shape
   *  \param in_right true if the hit occurs in the right shape
   *  \return true if the Intersection is allowed with those parameters
   */
  bool intersection_allowed(const SetOperation operation, const bool left_hit, const bool in_left, const bool in_right);
  
}

#endif

