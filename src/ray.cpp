
#include "ray.hpp"


namespace ray {

  Ray::Ray(const math::Tuple& ori, const math::Tuple& dir) :
    origin {ori}, direction {dir} {}

  math::Tuple Ray::position(const float time) const {

    return origin + (direction * time);
  }

  Ray Ray::transform(const math::Matrix& matrix) const {

    return Ray(matrix * origin , matrix * direction);
  }

  math::Tuple Ray::position(const float time) {

    // Instead of moving the shape from the center of the world, we can move the ray.
    // When we want to move the sphere, what we really want is to modify the distance between the ray and the sphere
    // --> Each transformation can be inversely applied to the ray instead of the object
    // We are converting points between object and world spaces
    
    return origin + (direction * time);
  }

  bool operator==(const Ray& ray1, const Ray& ray2) {
    return (ray1.origin == ray2.origin && ray1.direction == ray2.direction);
  }

  bool operator!=(const Ray& ray1, const Ray& ray2) {

    return !(ray1 == ray2);
  }
  
}

