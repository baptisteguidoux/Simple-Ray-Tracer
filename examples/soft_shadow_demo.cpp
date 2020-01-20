#include "camera.hpp"
#include "matrix.hpp"
#include "light.hpp"
#include "geo.hpp"
#include "world.hpp"

int main() {

  camera::Camera camera(400, 160, 0.7854);
  camera.transform = math::view_transform(math::Point(-3, 1, 2.5), math::Point(0, 0.5, 0), math::Vector(0, 1, 0));

  world::World world;
  
  auto light = std::make_shared<light::AreaLight>(math::Point(-1, 2, 4), math::Vector(2, 0, 0), 10, math::Vector(0, 2, 0), 10, color::Color(1.5, 1.5, 1.5));
  light->jitter_by = light::SequenceGenerator{0.3, 0.7, 0.9, 0.4};

  auto cube = std::make_shared<geo::Cube>();
  cube->material.color = color::Color(1.5, 1.5, 1.5);
  cube->material.ambient = 1;
  cube->material.diffuse = 0;
  cube->material.specular = 0;
  cube->material.cast_shadow = false;
  cube->transform = math::translation(0, 3, 4) * math::scaling(1, 1, 0.01);
  world.objects.push_back(cube);

  auto plane = std::make_shared<geo::Plane>();
  plane->material.color = color::WHITE;
  plane->material.ambient = 0.025;
  plane->material.diffuse = 0.67;
  plane->material.specular = 0;
  world.objects.push_back(plane);

  auto sphere1 = std::make_shared<geo::Sphere>();
  sphere1->transform = math::translation(0.5, 0.5, 0) * math::scaling(0.5, 0.5, 0.5);
  sphere1->material.color = color::Color(1, 0, 0);
  sphere1->material.ambient = 0.1;
  sphere1->material.specular = 0;
  sphere1->material.diffuse = 0.6;
  sphere1->material.reflective = 0.3;
  world.objects.push_back(sphere1);

  auto sphere2 = std::make_shared<geo::Sphere>();
  sphere2->transform = math::translation(-0.25, 0.33, 0) * math::scaling(0.33, 0.33, 0.33);
  sphere2->material.color = color::Color(0.5, 0.5, 1);
  sphere2->material.ambient = 0.1;
  sphere2->material.specular = 0;
  sphere2->material.diffuse = 0.6;
  sphere2->material.reflective = 0.3;
  world.objects.push_back(sphere2);

  //auto render = camera::render_threaded(camera, world);
  auto render = camera.render(world);
  render.write_ppm("/home/baptiste/soft_shadow_demo.ppm");
    
  return 0;
}
