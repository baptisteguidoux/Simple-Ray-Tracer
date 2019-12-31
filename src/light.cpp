#include <memory>
#include <cmath>

#include "tuple.hpp"
#include "light.hpp"
#include "geo.hpp"
#include "color.hpp"


namespace light {
  
  PointLight::PointLight(const math::Tuple& pos, const color::Color& int_) : position{pos}, intensity{int_} {}

  PointLight::PointLight() : position{math::Point(0, 0, 0)}, intensity{color::BLACK} {}

  bool operator==(const PointLight& first, const PointLight& second) {

    return (first.position == second.position) && (first.intensity == second.intensity);
  }

  bool operator!=(const PointLight& first, const PointLight& second) {

    return !(first == second);
  }    

  color::Color lighting(geo::Shape* object, const PointLight light, const math::Tuple& position,
  			const math::Tuple& eye_vector, const math::Tuple& normal_vector, const bool in_shadow) {

    // Use material or pattern as color    
    auto color = color::Color();
    if (object->material.pattern != nullptr)
      color = object->pattern_at(position);
    else
      color = object->material.color;
    
    // Combine the surface color with the light
    color::Color effective_color = color * light.intensity;

    // Direction to the light source
    auto light_vector = math::normalize(light.position - position);

    // Ambient contribution
    color::Color ambient = effective_color * object->material.ambient;

    // When the point is in shadow, the color is simply the ambient
    if (in_shadow)
      return ambient;

    color::Color diffuse;
    color::Color specular;
    // light_dot_nornal represents the cosine of the angle between the light vector and the normal vector
    // A negative number means the light is on the other side of the surface
    auto light_dot_normal = math::dot(light_vector, normal_vector);
    if (light_dot_normal < 0) {
      diffuse = color::Color(0, 0, 0);
      specular = color::Color(0, 0, 0);
    } else {
      // Difuse contribution
      diffuse = effective_color * object->material.diffuse * light_dot_normal;

      // reflect_dot_eye represents the cosine of the angle between the reflection vector and the eye vector
      // A negative number means the light reflects away from the eye
      auto reflect_vector = geo::reflect(-light_vector, normal_vector);
      double reflect_dot_eye = math::dot(reflect_vector, eye_vector);

      if (reflect_dot_eye < 0)
  	specular = color::Color(0, 0, 0);
      else {
        // Specular contribution
        double factor = pow(reflect_dot_eye, double{object->material.shininess});
  	specular = light.intensity * object->material.specular * factor;
      }
    }
  
    // Add the three contributions together to get the final shading
    return ambient + diffuse + specular;
  }  
  
}
