#include <cmath>
#include <vector>
#include <algorithm>
#include <future>
#include <thread>
#include <iostream>

#include "ray.hpp"
#include "tuple.hpp"
#include "world.hpp"


namespace world {

  World build_default_world() {
    
    auto wrld = World();
    wrld.light = light::PointLight(math::Point(-10.0, 10.0, -10.0), color::Color(1, 1, 1));

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

  bool World::contains_object(std::shared_ptr<geo::Shape> object) const {

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

  // bool World::is_point_shadowed(const math::Tuple& point) const {

  //   auto point_light_vec = light->position - point;
  //   auto point_light_magnitude = math::magnitude(point_light_vec);
  //   auto point_light_direction = math::normalize(point_light_vec);
    
  //   auto shadow_ray = ray::Ray(point, point_light_direction);
  //   auto intersections = intersects(shadow_ray);
  //   // Keep only the `intersections_with_objects_casting_shadows`, to avoid creating shadow under water for instance
  //   auto intersections_with_objects_casting_shadows = inter::Intersections(intersections.size());
  //   std::copy(intersections.begin(), intersections.end(), std::back_inserter(intersections_with_objects_casting_shadows));
  //   // std::copy_if(intersections.begin(), intersections.end(), intersections_with_objects_casting_shadows.begin(),
  //   // 		 [&](const inter::Intersection& ix) {return ix.geometry->material.cast_shadow == true;});    
  //   // if (intersections_with_objects_casting_shadows.size() == 0)
  //   //   return false;
    
  //   auto hit = inter::hit(intersections_with_objects_casting_shadows); // the hit is never negative, nothing to worry about intersections before the point

  //   // There is a hit, and it happens between the point and the light
  //   if (hit != std::nullopt && hit->t < point_light_magnitude)
  //     // No intersection, no shadow
  //     return true;

  //   // No hit, no shadow
  //   return false;
  // }

  // color::Color World::color_at(const ray::Ray& ry, const int remaining) const {

  //   // find the Intersections of the Ray and the World
  //   auto ixs = intersects(ry);
  //   // find the hit from the Intersections
  //   auto ht = inter::hit(ixs);
  //   // If no hit return black
  //   if (ht == std::nullopt)
  //     return color::BLACK;
    
  //   // Precompute the necessary values
  //   auto comps = prepare_computations(*ht, ry, ixs);

  //   return this->shade_hit(comps, remaining);
  // }

  // color::Color World::shade_hit(const inter::Computations& comps, const int remaining) const {

  //   bool is_shadowed = this->is_point_shadowed(comps.over_point);

  //   // If we want several lights, we should iterate over lights and sum the resulting Color
  //   auto surface = light::lighting(comps.geometry,
  // 				   *light, // std::optional
  // 				   comps.over_point, // ensure we are just above the surface, not below (floating point rounding errors...)
  // 				   comps.eye_vector,
  // 				   comps.normal_vector,
  // 				   is_shadowed);
    
  //   auto reflected = this->reflected_color(comps, remaining);
  //   auto refracted = this->refracted_color(comps, remaining);
    
  //   // If the surface's material is both reflective and refractive, use the Schlick approximation to get the reflectance
  //   if (comps.geometry->material.reflective > 0 && comps.geometry->material.transparency > 0) {
  //     auto reflectance = comps.schlick();
  //     return surface + reflected * reflectance + refracted * (1 - reflectance);
  //   } else
  // 	return surface + reflected + refracted;
  // }

  // color::Color World::reflected_color(const inter::Computations& comps, const int remaining) const {
    
  //   if (remaining <= 0)
  //     return color::BLACK;
    
  //   if (comps.geometry->material.reflective == 0)
  //     return color::BLACK;

  //   // use over_point, else risk intersect surface we reflect from
  //   auto reflect_ray = ray::Ray(comps.over_point, comps.reflect_vector);

  //   return color_at(reflect_ray, remaining - 1) * comps.geometry->material.reflective;
  // }

  // color::Color World::refracted_color(const inter::Computations& comps, const int remaining) const {

  //   if (remaining <= 0)
  //     return color::BLACK;
    
  //   if (comps.geometry->material.transparency == 0)
  //     return color::BLACK;

  //   // Deal with total internal reflection, when a ray will reflect off a surface instead of passing through.
  //   // It happens when the light is going to penetrate a new medium with an acute angle, and that old medidum refractive index > new medidum refractive index
  //   // Using Shell's Law that describes the relantionship between angle of incoming ray and angle of refracted ray
  //   auto n_ratio = comps.n1 / comps.n2;
  //   // ie cos(thetai)
  //   auto cos_i = math::dot(comps.eye_vector, comps.normal_vector);
  //   // ie sin(thetat)^2
  //   auto sin2_t = pow(n_ratio, 2) * (1 - pow(cos_i, 2));
  //   if(sin2_t > 1)
  //     return color::BLACK;

  //   // cos(theta t)
  //   auto cos_t = sqrt(1 - sin2_t);
  //   // direction of the refracted ray
  //   auto direction = comps.normal_vector * (n_ratio * cos_i - cos_t) - comps.eye_vector * n_ratio;

  //   auto refracted_ray = ray::Ray(comps.under_point, direction);

  //   // multiply by the transparency value to account for opacity
  //   return this->color_at(refracted_ray, remaining - 1) * comps.geometry->material.transparency;
  // }
  
  // Camera::Camera(const int h, const int v, const double fov) : hsize {h}, vsize {v}, field_of_view {fov} {
        
  //   // Determine pixel size
  //   // determine width of the half canvas
  //   double half_view = tan(field_of_view / 2.0);
  //   double aspect_ratio = static_cast<double>(hsize) / static_cast<double>(vsize);
  //   // wether it's a vertical or horizontal canvas
  //   if (aspect_ratio >= 1) {
  //     half_width = half_view;
  //     half_height = half_view / aspect_ratio;
  //   } else {
  //     half_width = half_view * aspect_ratio;
  //     half_height = half_view;
  //   }

  //   // full width of the canvas by the horizontal size of the canvas
  //   pixel_size = (half_width * 2.0) / hsize;
  // }
  
  // ray::Ray ray_for_pixel(const Camera& camera, const int px, const int py) {

  //   // Offset from the edge of the canvas to the pixel center
  //   auto xoffset = (px + 0.5) * camera.pixel_size;
  //   auto yoffset = (py + 0.5) * camera.pixel_size;

  //   // the untransformed coordinates of the pixel in world space
  //   // the camera looks toward -z
  //   auto world_x = camera.half_width - xoffset;
  //   auto world_y = camera.half_height - yoffset;

  //   // using the camera matrix, transform the canvas point and origin
  //   auto pixel = math::inverse(camera.transform) * math::Point(world_x, world_y, -1);
  //   auto origin = math::inverse(camera.transform) * math::Point(0, 0, 0);
  //   // compute ray's direction vector
  //   auto direction = math::normalize(pixel - origin);

  //   return ray::Ray(origin, direction);
  // }

  // canvas::Canvas render(const Camera& camera, const World& wrld) {

  //   auto image = canvas::Canvas(camera.hsize, camera.vsize);

  //   for (int y = 0; y < camera.vsize; y++)
  //     for (int x = 0; x < camera.hsize; x++) {
  // 	auto ry = ray_for_pixel(camera, x, y);
  // 	auto col = wrld.color_at(ry);
  // 	image.write_pixel(x, y, col);
  //     }

  //   return image;
  // }  
  
  // canvas::Canvas render_partial(const Camera& camera, const World& wrld, const int from_y, const int to_y) {

  //   auto image = canvas::Canvas(camera.hsize, to_y - from_y);
    
  //   for (int y = from_y; y < to_y; y++)
  //     for (int x = 0; x < camera.hsize; x++) {
  // 	auto ry = ray_for_pixel(camera, x, y);
  // 	auto col = wrld.color_at(ry);
  // 	image.write_pixel(x, y - from_y, col);
	
  //     }

  //   return image;
  // }

  // canvas::Canvas render_threaded(const Camera& camera, const World& wrld) {

  //   auto available_threads = std::thread::hardware_concurrency();
  //   if (available_threads == 1) {
  //     throw std::runtime_error {"Cannot use multithreading, only one thread available"};
  //   }

  //   std::cout << "Available threads: " << available_threads << '\n';
  //   auto working_threads = available_threads - 1;
  //   std::cout << "Using " << working_threads << " threads\n";

  //   using Task_type = canvas::Canvas(const Camera&, const World&, const int, const int);

  //   // packaged_task simplify the setting of tasks that use futures to run on threads
  //   std::vector<std::packaged_task<Task_type>> render_tasks(working_threads);
  //   for (auto& render_task : render_tasks)
  //     render_task = std::packaged_task<Task_type> {render_partial};

  //   // futures avoid returning by reference
  //   std::vector<std::future<canvas::Canvas>> canvas_futures(working_threads);
  //   for (size_t i = 0; i < render_tasks.size(); i++)
  //     canvas_futures[i] = render_tasks[i].get_future();

  //   // The image to render is divided horizontally, each thread being in charge of one section
  //   unsigned int thread_render_size = camera.vsize / working_threads;

  //   std::vector<std::thread> threads;
  //   for (size_t i = 0; i < render_tasks.size(); i++)
  //     threads.push_back(std::thread {std::move(render_tasks[i]),
  // 	                std::ref(camera), std::ref(wrld),
  // 	                thread_render_size * i, thread_render_size * i + thread_render_size});

  //   // Wait for all the threads to terminate
  //   std::for_each(threads.begin(), threads.end(), [&](std::thread& thread){thread.join();});

  //   // concatenate the canvas to obtain final image
  //   auto result = canvas::Canvas(camera.hsize, 0);
  //   for (auto& canvas_future : canvas_futures)
  //     result = result + canvas_future.get();
    
  //   return result;
  // }
  
}


