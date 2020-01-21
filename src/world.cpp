#include <cmath>
#include <vector>
#include <algorithm>

#include "ray.hpp"
#include "tuple.hpp"
#include "world.hpp"


namespace world {

  World build_default_world() {
    
    auto wrld = World();
    wrld.light = std::make_shared<light::PointLight>(math::Point(-10.0, 10.0, -10.0), color::Color(1, 1, 1));

    auto sphere1 = std::make_shared<geo::Sphere>();
    sphere1->material.color = color::Color(0.8, 1.0, 0.6);
    sphere1->material.diffuse = 0.7;
    sphere1->material.specular = 0.2;
	
    auto sphere2 = std::make_shared<geo::Sphere>();
    sphere2->transform = math::scaling(0.5, 0.5, 0.5);

    wrld.objects.push_back(sphere1);
    wrld.objects.push_back(sphere2);    

    return wrld;
  }

  bool World::contains_object(geo::Shape* object) const {

    auto it = std::find_if(objects.begin(), objects.end(), [&](const std::shared_ptr<geo::Shape> shp) {return *shp == *object;});

    if (it != objects.end())
      return true;
      
    return false;
  }

  geo::Intersections World::intersects(const ray::Ray& ry) const {

    auto result = geo::Intersections();
    
    for (const auto& obj : objects) {
      auto xs = obj->intersects(ry);
      result.insert(result.begin(), xs.begin(), xs.end());
    }

    // sort result
    std::sort(result.begin(), result.end(), [&](const geo::Intersection& first, const geo::Intersection& second) {return first.t < second.t;});

    return result;
  }

  bool World::is_shadowed(const math::Tuple& light_position, const math::Tuple& point) const {

    auto point_light_vec = light_position - point;
    auto point_light_magnitude = math::magnitude(point_light_vec);
    auto point_light_direction = math::normalize(point_light_vec);
    
    auto shadow_ray = ray::Ray(point, point_light_direction);
    auto intersections = intersects(shadow_ray);
    
    // Keep only the `intersections_with_objects_casting_shadows`, to avoid creating shadow under water for instance
    geo::Intersections intersections_with_objects_casting_shadows;
    for (const auto& inter : intersections)
      if (inter.geometry->material.cast_shadow)
    	intersections_with_objects_casting_shadows.push_back(inter);

    if (intersections_with_objects_casting_shadows.size() == 0)
      return false;
    
    auto hit = geo::hit(intersections_with_objects_casting_shadows); // the hit is never negative, nothing to worry about intersections before the point

    // There is a hit, and it happens between the point and the light
    if (hit != std::nullopt && hit->t < point_light_magnitude)
      return true;

    // No hit, no shadow
    return false;
  }
  
  color::Color World::shade_hit(const geo::Computations& comps, const int remaining) {

    float intensity = light->intensity_at(comps.over_point, *this);
    
    // If we want several lights, we should iterate over lights and sum the resulting Color
    auto surface = light->lighting(comps.geometry.get(),
    				   comps.over_point, // ensure we are just above the surface, not below (floating point rounding errors...)
    				   comps.eye_vector,
    				   comps.normal_vector,
    				   intensity);
    
    auto reflected = reflected_color(comps, remaining);
    auto refracted = refracted_color(comps, remaining);

    // If the surface's material is both reflective and refractive, use the Schlick approximation to get the reflectance
    if (comps.geometry->material.reflective > 0 && comps.geometry->material.transparency > 0) {
      auto reflectance = comps.schlick();
      return surface + reflected * reflectance + refracted * (1 - reflectance);
    } else
      return surface + reflected + refracted;
  }

  color::Color World::color_at(const ray::Ray& ry, const int remaining) {

    // find the Intersections of the Ray and the World
    auto ixs = intersects(ry);
    // find the hit from the Intersections
    auto ht = geo::hit(ixs);
    // If no hit return black
    if (ht == std::nullopt)
      return color::BLACK;
    
    // Precompute the necessary values
    auto comps = geo::prepare_computations(*ht, ry, ixs);

    return shade_hit(comps, remaining);
  }  

  color::Color World::reflected_color(const geo::Computations& comps, const int remaining) {
     
    if (remaining <= 0)
      return color::BLACK;
    
    if (comps.geometry->material.reflective == 0)
      return color::BLACK;

    // use over_point, else risk intersect surface we reflect from
    auto reflect_ray = ray::Ray(comps.over_point, comps.reflect_vector);

    return color_at(reflect_ray, remaining - 1) * comps.geometry->material.reflective;
  }

  color::Color World::refracted_color(const geo::Computations& comps, const int remaining) {

    if (remaining <= 0)
      return color::BLACK;
    
    if (comps.geometry->material.transparency == 0)
      return color::BLACK;

    // Deal with total internal reflection, when a ray will reflect off a surface instead of passing through.
    // It happens when the light is going to penetrate a new medium with an acute angle, and that old medidum refractive index > new medidum refractive index
    // Using Shell's Law that describes the relantionship between angle of incoming ray and angle of refracted ray
    auto n_ratio = comps.n1 / comps.n2;
    // ie cos(thetai)
    auto cos_i = math::dot(comps.eye_vector, comps.normal_vector);
    // ie sin(thetat)^2
    auto sin2_t = pow(n_ratio, 2) * (1 - pow(cos_i, 2));
    if(sin2_t > 1)
      return color::BLACK;

    // cos(theta t)
    auto cos_t = sqrt(1 - sin2_t);
    // direction of the refracted ray
    auto direction = comps.normal_vector * (n_ratio * cos_i - cos_t) - comps.eye_vector * n_ratio;

    auto refracted_ray = ray::Ray(comps.under_point, direction);

    // multiply by the transparency value to account for opacity
    return this->color_at(refracted_ray, remaining - 1) * comps.geometry->material.transparency;
  }  
}

