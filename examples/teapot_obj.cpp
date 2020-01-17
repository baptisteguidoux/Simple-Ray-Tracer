#include "parser.hpp"
#include "world.hpp"
#include "light.hpp"
#include "camera.hpp"
#include "tuple.hpp"

#include <iostream>
#include <cmath>

void triangle_iterate(std::vector<std::shared_ptr<geo::Shape>> shapes) {

  for (auto sh : shapes) {
    auto tri = dynamic_cast<geo::SmoothTriangle*>(sh.get());
    if (tri != nullptr)
      std::cout << tri->p1 << ", " << tri->p2 << ", " << tri->p3 << ", " << tri->n1 << ", " << tri->n2 << ", " << tri->n3 << '\n';
    else {
      auto gr = dynamic_cast<geo::Group*>(sh.get());
      if (gr != nullptr)
	triangle_iterate(gr->shapes);
    }
  }
  
}

int main() {


  parser::ObjParser parser("/datas/guiba/packages/perso/Cpp/Simple-Ray-Tracer/examples/Teapot.obj");

  auto teapot = parser.to_group();
  teapot->transform = math::translation(0, -1, 1) * math::scaling(0.05, 0.05, 0.05) * math::rotation_y(M_PI);
  teapot->divide(5);
  
  //triangle_iterate(teapot->shapes);


  auto wrld = world::World();
  wrld.light = light::PointLight(math::Point(-10, 10, -10), color::Color(1, 1, 1));

  wrld.objects.push_back(std::move(teapot));
  // for (auto sh : teapot->shapes) {
  //   sh->transform = math::translation(0, -1, 1) * math::scaling(0.05, 0.05, 0.05) * math::rotation_y(M_PI);
  //   wrld.objects.push_back(sh);
  // }
  
  auto camera = camera::Camera(200, 100, M_PI / 3);
  //auto camera = camera::Camera(1000, 500, M_PI / 3);
  camera.transform = math::view_transform(math::Point(0, 2, -5), math::Point(0, 0, 0), math::Vector(0, 1, 0));

  std::cout << "world objects: " << wrld.objects.size() << '\n';
  geo::Group* child = dynamic_cast<geo::Group*>(wrld.objects[0].get());
  std::cout << "object size: " << child->shapes.size() << '\n';
  // geo::Group* subchild = dynamic_cast<geo::Group*>(child->shapes[0].get());
  // std::cout << "subobject size: " << subchild->shapes.size() << '\n';
  auto image = camera::render_threaded(camera, wrld);
  image.write_ppm("/datas/guiba/teapot_test.ppm");
  
  return 0;
}

