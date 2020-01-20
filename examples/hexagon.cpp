#include <cmath>
#include <memory>

#include "tuple.hpp"
#include "world.hpp"
#include "geo.hpp"
#include "color.hpp"
#include "light.hpp"
#include "camera.hpp"

std::shared_ptr<geo::Sphere>build_hex_corner() {

  auto corner = std::make_shared<geo::Sphere>();
  corner->transform = math::translation(0, 0, -1) * math::scaling(0.25, 0.25, 0.25);
  return corner;
}

std::shared_ptr<geo::Cylinder> build_hex_cyl() {
  
  auto edge = std::make_shared<geo::Cylinder>();
  edge->minimum = 0;
  edge->maximum = 1;
  edge->transform = math::translation(0, 0, -1) * math::rotation_y(- M_PI / 6) *
                    math::rotation_z(- M_PI / 2) * math::scaling(0.25, 1, 0.25);

  return edge;
}
std::shared_ptr<geo::Group> build_hex_edge() {

  auto side = std::make_shared<geo::Group>();
  side->add_child(build_hex_corner());
  side->add_child(build_hex_cyl());

  return side;
}


int main() {

  auto wrld = world::World();
  // white light source, from above and the left
  wrld.light = std::make_shared<light::PointLight>(math::Point(0, 7, -1), color::Color(1, 1, 1));

  auto hexagon = std::make_shared<geo::Group>();
  
  for (int i = 0; i < 6; i++) {
    auto side =  build_hex_edge();
    side->transform = math::rotation_y(i * M_PI / 3);
    //wrld.objects.push_back(side);
    hexagon->add_child(side);
  }

  wrld.objects.push_back(hexagon);

  auto camera = camera::Camera(400, 200, M_PI / 3);
  camera.transform = math::view_transform(math::Point(0, 1.5, -4), math::Point(0, 0.5, 0), math::Vector(0, 1, 0));

  auto image = camera::render_threaded(camera, wrld);
  image.write_ppm("/home/baptiste/hexagon.ppm");
  return 0;
}

