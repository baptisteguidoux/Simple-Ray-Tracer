
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
  
}

