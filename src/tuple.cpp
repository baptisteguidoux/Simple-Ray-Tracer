#include <cmath>
#include <ostream>
#include <stdexcept>

#include "tuple.hpp"
//#include "utils.h"


namespace math {

  Tuple::Tuple(double x_, double y_, double z_, double w_) : x{x_}, y{y_}, z{z_}, w{w_} {};

  // CBB: Check if can implement a faster subscript with pointers to member variables
  double& Tuple::operator[] (const size_t i) {

    switch(i){
    case 0 : return x;
    case 1 : return y;
    case 2 : return z;
    case 3 : return w;
    default : throw std::runtime_error {"Bad Tuple index"};
    }
  }

  const double& Tuple::operator[] (const size_t i) const {

    switch(i){
    case 0 : return x;
    case 1 : return y;
    case 2 : return z;
    case 3 : return w;
    default : throw std::runtime_error {"Bad Tuple index"};
    }
  }

  Point::Point(double x_, double y_, double z_) : Tuple(x_, y_, z_, 1.0) {}

  Tuple& Point::operator=(const Tuple& source) {

    x = source.x;
    y = source.y;
    z = source.z;
    w = source.w;

    return *this;
  }

  Vector::Vector(double x_, double y_, double z_) : Tuple(x_, y_, z_, 0.0) {}

  Tuple& Vector::operator=(const Tuple& source) {

    x = source.x;
    y = source.y;
    z = source.z;
    w = source.w;

    return *this;
  }

  // Operator overloaded functions
  bool operator==(const Tuple& first, const Tuple& second) {

    return (almost_equal(first.x, second.x) && almost_equal(first.y, second.y)
	    && almost_equal(first.z, second.z) && almost_equal(first.w, second.w));
  }

  bool operator!=(const Tuple& first, const Tuple& second) {

    return !(first == second);
  }

  Tuple operator+(const Tuple& first, const Tuple& second) {

    return Tuple(first.x + second.x, first.y + second.y, first.z + second.z, first.w + second.w);
  }

  Tuple operator-(const Tuple& first, const Tuple& second) {

    /* Substracting point1 - point2 gives us the vector pointing from point2 to point1
       Substracting point1 - vector1 gives us a new point, moving backwards from the given point
       Substracting vector1 - vector2 gives us a new vector representing the change in direction between the two
     */

    return Tuple(first.x - second.x, first.y - second.y, first.z - second.z, first.w - second.w);
  }

  Tuple operator-(const Tuple& tuple) {

    return Tuple(-tuple.x, -tuple.y, -tuple.z, -tuple.w);
  }

  Tuple operator*(const Tuple& tuple, const double scalar) {

    return Tuple(tuple.x * scalar, tuple.y * scalar, tuple.z * scalar, tuple.w * scalar);
  }

  Tuple operator/(const Tuple& tuple, const double scalar) {

    return Tuple(tuple.x / scalar, tuple.y / scalar, tuple.z / scalar, tuple.w / scalar);
  }

  bool is_point(const Tuple& tuple) {

    return (almost_equal(tuple.w, 1.0));
  }

  bool is_vector(const Tuple& tuple) {

    return (almost_equal(tuple.w, 0.0));
  }

  double magnitude(const Tuple& tuple) {

    // Note: when vector.magnitude == 1, this is a `unit vector`

    return sqrt(pow(tuple.x, 2) + pow(tuple.y, 2) + pow(tuple.z, 2) + pow(tuple.w, 2));
  }

  Tuple normalize(const Tuple& tuple) {

    // To ensure that the calculations of ray vectors or surface normals are at the same scale, we normalize vectors
    // To normalize a vector, we divide each of its component by vector's magnitude

    return tuple / magnitude(tuple);
  }

  double dot(const Tuple& first, const Tuple& second) {

    /*  The smaller the dot product, the larger the angle between the vectors
	For example, a dot product of 1 means vectors are identical, a dot product of -1 means they point in opposite directions
	The dot product is actually the cosine of the angle between the vectors
	NB: https://betterexplained.com/articles/vector-calculus-understanding-the-dot-product/
    */

    return first.x * second.x + first.y * second.y + first.z * second.z + first.w * second.w;
  }

  Tuple cross(const Tuple& first, const Tuple& second) {


    if (is_point(first) || is_point(second)) {
      throw std::runtime_error("Cannot cross product on 4D tuples");
    }

    return Tuple(first.y * second.z - first.z * second.y,
		      first.z * second.x - first.x * second.z,
		      first.x * second.y - first.y * second.x,
		      0);
  }

  std::ostream& operator<<(std::ostream& os, const Tuple& tuple) {

    os << "(" << tuple.x << ", " <<tuple.y << ", " << tuple.z << ", " << tuple.w << ")";

    return os;
  }

  bool almost_equal(const double a, const double b) {
    return std::abs(a - b) < EPSILON;
  };

  double map(const double value,
	     const double src_range_start, const double src_range_end,
	     const double dest_range_start, const double dest_range_end) {

    return (dest_range_start + (dest_range_end - dest_range_start) * ((value - src_range_start) / (src_range_end - src_range_start)));
  }

}

