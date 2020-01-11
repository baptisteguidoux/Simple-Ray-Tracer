/*! \file tuple.hpp
 */

#ifndef TUPLE_H
#define TUPLE_H


#include <iostream>

/*! \namespace math
 */
namespace math {


  /* UTILS */

  /*! \fn almost_equal(const double a, const double b
   *  \brief To handle equality with floating point values
   *  \param a a num
   *  \param b another num
   *  \return true if the diff between the two abs values is less than epsilon
   */
  bool almost_equal(const double a, const double b);

  /*! \fn double map(const double value, const double src_range_start, const double src_range_end, const double dest_range_start, const double dest_range_end)
   *  \brief Transpose a value from a source range to a dest range of values
   *  \param value the value to transpose
   *  \param src_range_start The start value of the source range
   *  \param src_range_end The end value of the source range
   *  \param dest_range_start The start value of the destination range
   *  \param dest_range_end The end value of the destination range
   *  \return the transposed value
   */
  double map(const double value,
	     const double src_range_start, const double src_range_end,
	     const double dest_range_start, const double dest_range_end);

  /* TUPLE */

  /*! \struct Tuple
   *  A collection of 4 double
   */
  struct Tuple {

    double x;
    double y;
    double z;
    double w;

    /* Tuple constructor
     */
    Tuple(double x_, double y_, double z_, double w_);

    /* Tuple default constructor (0 for each component)
     */    
    Tuple() = default;

    /*! \fn double& operator[](const size_t i)
     *  \brief Access elements of the Tuple using an index
     *  \param i index of the element
     *  \return the element at the given index
     */
    double& operator[](const size_t i);

    /*! \fn const double& operator[](const size_t i) const
     *  \brief Access elements of the Tuple using an index
     *  \param i index of the element
     *  \return the element at the given index
     */
    const double& operator[](const size_t i) const;
  };

  /*! \struct Point
   *  \brief A collection of 4 double, the 4th one being initialized at 1
   */
  struct Point : Tuple {

    /* Point constructor
     */
    Point(double x_, double y_, double z_);

    /* Point default constructor
     */    
    Point() = default;

    /*! \fn Tuple& operator=(const Tuple& source)
     *  \brief Copy operator
     */
    Tuple& operator=(const Tuple& source);
  };

  /*! \struct Vector
   *  \brief A collection of 4 double, the 4th one being initialized at 0
   */
  struct Vector : Tuple {

    /* Vector constructor
     */
    Vector(double x_, double y_, double z_);

    /* Vector default constructor
     */    
    Vector() = default;

    /*! \fn Tuple& operator=(const Tuple& source)
     *  \brief Copy operator
     */
    Tuple& operator=(const Tuple& source);
  };

  /*! \fn operator==(const Tuple& first, const Tuple& second)
   *  \brief Tuple equality comparison function
   *  \param first a Tuple
   *  \param second another Tuple
   *  \return true if the two Tuple are equal
   */
  bool operator==(const Tuple& first, const Tuple& second);

  /*! \fn operator!=(const Tuple& first, const Tuple& second)
   *  \brief Tuple unequality comparison function
   *  \param first a Tuple
   *  \param second another Tuple
   *  \return true if the two Tuple are not equal
   */
  bool operator!=(const Tuple& first, const Tuple& second);

  /*! \fn operator+(const Tuple& first, const Tuple& second)
   *  \brief Tuple addition operation
   *  \param first a Tuple
   *  \param second another Tuple
   *  \return A new Tuple, the sum of the two inputs
   */
  Tuple operator+(const Tuple& first, const Tuple& second);

  /*! \fn operator-(const Tuple& first, const Tuple& second)
   *  \brief Tuple substraction operation
   *  \param first a Tuple
   *  \param second another Tuple
   *  \return A new Tuple, the difference of the two inputs
   */
  Tuple operator-(const Tuple& first, const Tuple& second);

  /*! \fn operator-(const Tuple& tuple)
   *  \brief Tuple negation
   *  \param tuple a Tuple
   *  \return A new Tuple, the negated input
   */
  Tuple operator-(const Tuple& tuple);

  /*! \fn operator*(const Tuple& tuple, const double scalar)
   *  \brief Tuple multiplication operation
   *  \param tuple a Tuple
   *  \param scalar a scalar value
   *  \return A new Tuple, the multiplication of the tuple by the scalar
   */
  Tuple operator*(const Tuple& tuple, const double scalar);

  /*! \fn operator/(const Tuple& tuple, const double scalar)
   *  \brief Tuple division operation
   *  \param tuple a Tuple
   *  \param scalar scalar a scalar value
   *  \return A new Tuple, the division of the tuple by the scalar
   */
  Tuple operator/(const Tuple& tuple, const double scalar);

  /*! \fn is_point(const Tuple& tuple)
   *  \brief Tuple is point predicate function
   *  \param tuple a Tuple
   *  \return true if the Tuple is a point (w = 1)
   */
  bool is_point(const Tuple& tuple);

  /*! \fn is_vector(const Tuple& tuple)
   *  \brief Tuple is vector predicate function
   *  \param tuple a Tuple
   *  \return true if the Tuple is a vector (w = 0)
   */
  bool is_vector(const Tuple& tuple);

  /*! \fn is_vector(const Tuple& tuple)
   *  \brief Tuple is vector predicate function
   *  \param tuple a Tuple
   *  \return true if the Tuple is a vector (w = 0)
   */
  bool is_vector(const Tuple& tuple);

  /*! \fn magnitude(const Tuple& tuple)
   *  \brief Calculate the magnitude of the Tuple (size of the vector)
   *  \param tuple a Tuple
   *  \return the magnitude
   */  
  double magnitude(const Tuple& tuple);

  /*! \fn normalize(const Tuple& tuple)
   *  \brief Normalize the given vector (a unit vector)
   *  \param tuple a Tuple
   *  \return the normalized tuple
   */
  Tuple normalize(const Tuple& tuple);

  /*! \fn dot(const Tuple& first, const Tuple& second)
   *  \brief Tuple dot product
   *  \param first a Tuple
   *  \param second another Tuple
   *  \return A scalar, result ot the dot product
   */  
  double dot(const Tuple& first, const Tuple& second);


  /*! \fn cross(const Tuple& first, const Tuple& second)
   *  \brief Tuple cross product
   *  \param first a Tuple
   *  \param second another Tuple
   *  \return A new Tuple, result of the cross product
   */    
  Tuple cross(const Tuple& first, const Tuple& second);

  /*! \fn std::ostream& operator<<(std::ostream& os, const Tuple& tuple)
   *  \brief Format and output a Tuple
   *  \param os output stream
   *  \param tuple a Tuple
   *  \return an output stream
   */
  std::ostream& operator<<(std::ostream& os, const Tuple& tuple);

}

#endif


#ifndef RAY_MATH_CONSTANTS
#define RAY_MATH_CONSTANTS

namespace math {

  const static double EPSILON = 0.00001; /*<! For floating point values comparison */
}

#endif

