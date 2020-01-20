#include "parser.hpp"
#include "world.hpp"
#include "light.hpp"
#include "camera.hpp"
#include "tuple.hpp"

#include <iostream>
#include <cmath>


int main() {

  parser::ObjParser parser("/home/baptiste/dev/Simple-Ray-Tracer/examples/Teapot.obj");

  auto teapot = parser.to_group();
  teapot->transform = math::translation(0, -1, 1) * math::scaling(0.05, 0.05, 0.05) * math::rotation_y(M_PI);

  auto wrld = world::World();
  wrld.light = std::make_shared<light::PointLight>(math::Point(-10, 10, -10), color::Color(1, 1, 1));

  wrld.objects.push_back(teapot);
  
  auto camera = camera::Camera(200, 100, M_PI / 3);
  //auto camera = camera::Camera(1000, 500, M_PI / 3);
  camera.transform = math::view_transform(math::Point(0, 0, -5), math::Point(0, 0, 0), math::Vector(0, 1, 0));

  std::cout << "world objects: " << wrld.objects.size() << '\n';
  geo::Group* child = dynamic_cast<geo::Group*>(wrld.objects[0].get());
  std::cout << "object size: " << child->shapes.size() << '\n';
  auto image = camera::render_threaded(camera, wrld);
  image.write_ppm("/home/baptiste/test.ppm");
  
  return 0;
}

