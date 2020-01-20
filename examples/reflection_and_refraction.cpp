#include <cmath>
#include <memory>

#include "tuple.hpp"
#include "world.hpp"
#include "geo.hpp"
#include "color.hpp"
#include "light.hpp"
#include "material.hpp"
#include "camera.hpp"


// # This file describes the scene illustrated at the start
// # of chapter 11, "Reflection and Refraction", in "The
// # Ray Tracer Challenge"
// #
// # by Jamis Buck <jamis@jamisbuck.org>
// source: http://forum.raytracerchallenge.com/thread/4/reflection-refraction-scene-description


int main() {

  auto w = world::World();
  
  auto wall_material = material::Material();
  wall_material.pattern = std::make_shared<pattern::StripePattern>(color::Color(0.45, 0.45, 0.45), color::Color(0.55, 0.55, 0.55));
  wall_material.pattern->transform = math::rotation_y(1.5708) * math::scaling(0.25, 0.25, 0.25);
  wall_material.ambient = 0;  
  wall_material.diffuse = 0.4;
  wall_material.specular = 0;
  wall_material.reflective = 0.3;

  auto checkered_floor = std::make_shared<geo::Plane>();
  checkered_floor->transform = math::rotation_y(0.31415);
  checkered_floor->material.pattern = std::make_shared<pattern::CheckerPattern>(color::Color(0.35, 0.35, 0.35), color::Color(0.65, 0.65, 0.65));
  checkered_floor->material.specular = 0;
  checkered_floor->material.reflective = 0.4;
  w.objects.push_back(checkered_floor);

  auto ceiling = std::make_shared<geo::Plane>();
  ceiling->transform = math::translation(0, 5, 0);
  ceiling->material.color = color::Color(0.8, 0.8, 0.8);
  ceiling->material.ambient = 0.3;
  ceiling->material.specular = 0;
  w.objects.push_back(ceiling);

  auto west_wall = std::make_shared<geo::Plane>();
  west_wall->transform = math::translation(-5, 0, 0) * math::rotation_z(1.5708) *  math::rotation_y(1.5708);
  west_wall->material = wall_material;
  w.objects.push_back(west_wall);

  auto east_wall = std::make_shared<geo::Plane>();
  east_wall->transform = math::translation(5, 0, 0) * math::rotation_z(1.5708) * math::rotation_y(1.5708);
  east_wall->material = wall_material;
  w.objects.push_back(east_wall);

  auto north_wall = std::make_shared<geo::Plane>();
  north_wall->transform =  math::translation(0, 0, 5)    * math::rotation_x(1.5708);
  north_wall->material = wall_material;  
  w.objects.push_back(north_wall);
  
  auto south_wall = std::make_shared<geo::Plane>();
  south_wall->transform = math::translation(0, 0, -5) * math::rotation_x(1.5708);
  south_wall->material = wall_material;  
  w.objects.push_back(south_wall);
  
  // background balls
  
  auto sphere1 = std::make_shared<geo::Sphere>();
  sphere1->transform = math::translation(4.6, 0.4, 1) * math::scaling(0.4, 0.4, 0.4); 
  sphere1->material.color = color::Color(0.8, 0.5, 0.3);
  sphere1->material.shininess = 50;
  w.objects.push_back(sphere1);
  
  auto sphere2 = std::make_shared<geo::Sphere>();
  sphere2->transform = math::translation(4.7, 0.3, 0.4) * math::scaling(0.3, 0.3, 0.3);
  sphere2->material.color = color::Color(0.9, 0.4, 0.5);
  sphere2->material.shininess = 50;
  w.objects.push_back(sphere2);

  auto sphere3 = std::make_shared<geo::Sphere>();
  sphere3->transform = math::translation(-1, 0.5, 4.5) * math::scaling(0.5, 0.5, 0.5);
  sphere3->material.color = color::Color(0.4, 0.9, 0.6);
  sphere3->material.shininess = 50;
  w.objects.push_back(sphere3);
  
  auto sphere4 = std::make_shared<geo::Sphere>();
  sphere4->transform =  math::translation(-1.7, 0.3, 4.7) * math::scaling(0.3, 0.3, 0.3);
  sphere4->material.color = color::Color(0.4, 0.6, 0.9);
  sphere4->material.shininess = 50;
  w.objects.push_back(sphere4);
  
  // foreground balls

  auto red_sphere = std::make_shared<geo::Sphere>();
  red_sphere->transform = math::translation(-0.6, 1, 0.6);
  red_sphere->material.color = color::Color(1, 0.3, 0.2);
  red_sphere->material.specular = 0.4;
  red_sphere->material.shininess = 5;
  w.objects.push_back(red_sphere);

  auto blue_sphere = std::make_shared<geo::Sphere>();
  blue_sphere->transform = math::translation(0.6, 0.7, -0.6) * math::scaling(0.7, 0.7, 0.7);
  blue_sphere->material.color = color::Color(0, 0, 0.2);
  blue_sphere->material.ambient = 0;
  blue_sphere->material.diffuse = 0.4;
  blue_sphere->material.specular = 0.9;
  blue_sphere->material.shininess = 300;
  blue_sphere->material.reflective = 0.9;
  blue_sphere->material.transparency = 0.9;
  blue_sphere->material.refractive_index = 1.5;
  w.objects.push_back(blue_sphere);

  auto green_sphere = std::make_shared<geo::Sphere>();
  green_sphere->transform = math::translation(-0.7, 0.5, -0.8) * math::scaling(0.5, 0.5, 0.5);
  green_sphere->material.color = color::Color(0, 0.2, 0);
  green_sphere->material.ambient = 0;
  green_sphere->material.diffuse = 0.4;
  green_sphere->material.specular = 0.9;
  green_sphere->material.shininess = 300;
  green_sphere->material.reflective = 0.9;
  green_sphere->material.transparency = 0.9;
  green_sphere->material.refractive_index = 1.5;
  w.objects.push_back(green_sphere);

  w.light = std::make_shared<light::PointLight>(math::Point(-4.9, 4.9, -1), color::Color(1, 1, 1));

  auto camera = camera::Camera(400, 200, 1.152);
  camera.transform = math::view_transform(math::Point(-2.6, 1.5, -3.9), math::Point(-0.6, 1, -0.8), math::Vector(0, 1, 0));
  //                                      ^ from                        ^ up                        ^ to
  
  auto image = camera::render_threaded(camera, w);
  image.write_ppm("/home/baptiste/reflection_and_refraction.ppm");  

  return 0;
}
