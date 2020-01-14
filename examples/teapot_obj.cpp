#include "parser.hpp"
#include "world.hpp"
#include "light.hpp"
#include "camera.hpp"
#include "tuple.hpp"

#include <iostream>

int main() {

  parser::ObjParser parser("/home/baptiste/dev/Simple-Ray-Tracer/examples/teapot-low.obj");

  auto teapot = parser.to_group();
  //teapot->transform = math::scaling(0.01, 0.01, 0.01);
  teapot->transform = math::translation(0, -5, 15);
  teapot->divide(5);

  auto wrld = world::World();
  wrld.light = light::PointLight(math::Point(-10, 10, -10), color::Color(1, 1, 1));

  wrld.objects.push_back(std::move(teapot));
  
  auto camera = camera::Camera(150, 75, M_PI / 3);
  //auto camera = camera::Camera(1000, 500, M_PI / 3);
  camera.transform = math::view_transform(math::Point(0, 1.5, -25), math::Point(0, 1, 0), math::Vector(0, 1, 0));

  std::cout << "world objects: " << wrld.objects.size() << '\n';
  geo::Group* child = dynamic_cast<geo::Group*>(wrld.objects[0].get());
  std::cout << "object size: " << child->shapes.size() << '\n';
  // geo::Group* subchild = dynamic_cast<geo::Group*>(child->shapes[0].get());
  // std::cout << "subobject size: " << subchild->shapes.size() << '\n';
  auto image = camera::render_threaded(camera, wrld);
  image.write_ppm("/home/baptiste/teapot_low.ppm");
  
  return 0;
}

