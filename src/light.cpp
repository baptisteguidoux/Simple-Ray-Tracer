#include <memory>
#include <cmath>

#include "tuple.hpp"
#include "light.hpp"
#include "geo.hpp"
#include "color.hpp"
#include "world.hpp"


namespace light {
  
  PointLight::PointLight(const math::Tuple& pos, const color::Color& int_) : position{pos}, intensity{int_} {}

  PointLight::PointLight() : position{math::Point(0, 0, 0)}, intensity{color::BLACK} {}

  
  float PointLight::intensity_at(const math::Tuple& point, const world::World& wrld) const {

    if (wrld.is_shadowed(position, point))
      return 0.0;

    return 1.0;
  }

  AreaLight::AreaLight(const math::Tuple& corner_, const math::Tuple& uvec_, const uint usteps_,
		       const math::Tuple& vvec_, const uint vsteps_, const color::Color& intensity_)
    : corner {corner_}, uvec {uvec_ / usteps_}, usteps {usteps_}, vvec {vvec_ / vsteps_}, vsteps {vsteps_},
      samples {usteps_ * vsteps_}, position {(uvec_ + vvec_) / 2}, intensity {intensity_} {
	position.w = 1; // to Point
      }

  math::Tuple AreaLight::point_at(const uint u, const uint v) const {

    return corner + uvec * (u + 0.5) + vvec * (v + 0.5);
  }

  float AreaLight::intensity_at(const math::Tuple& point, const world::World& wrld) const {

    float intensity = 0;
    for (uint u = 0; u < usteps; u++)
      for (uint v = 0; v < vsteps; v++) {
	auto light_pos = point_at(u, v);
	if (! wrld.is_shadowed(light_pos, point))
	  intensity += 1;
      }

    return intensity / samples;
  }

  bool operator==(const PointLight& first, const PointLight& second) {

    return (first.position == second.position) && (first.intensity == second.intensity);
  }

  bool operator!=(const PointLight& first, const PointLight& second) {

    return !(first == second);
  }    

  color::Color lighting(geo::Shape* object, const PointLight light, const math::Tuple& position,
  			const math::Tuple& eye_vector, const math::Tuple& normal_vector, const float intensity) {

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
    if (math::almost_equal(intensity, 0.0))
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
      diffuse = effective_color * object->material.diffuse * light_dot_normal * intensity;

      // reflect_dot_eye represents the cosine of the angle between the reflection vector and the eye vector
      // A negative number means the light reflects away from the eye
      auto reflect_vector = geo::reflect(-light_vector, normal_vector);
      double reflect_dot_eye = math::dot(reflect_vector, eye_vector);

      if (reflect_dot_eye < 0)
  	specular = color::Color(0, 0, 0);
      else {
        // Specular contribution
        double factor = pow(reflect_dot_eye, double{object->material.shininess});
  	specular = light.intensity * object->material.specular * factor * intensity;
      }
    }
  
    // Add the three contributions together to get the final shading
    return ambient + diffuse + specular;
  }  
  
}
