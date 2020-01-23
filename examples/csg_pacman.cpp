#include <cmath>
#include <memory>

#include "tuple.hpp"
#include "world.hpp"
#include "geo.hpp"
#include "csg.hpp"
#include "color.hpp"
#include "light.hpp"
#include "camera.hpp"


int main() {
  
  auto wrld = world::World();
  wrld.light = std::make_shared<light::PointLight>(math::Point(-10, 10, -10), color::Color(1, 1, 1));

  auto sphere = std::make_shared<geo::Sphere>();
  sphere->transform = math::scaling(1.4, 1.4, 1.4);
  sphere->material.color = color::Color(1, 0.95, 0);

  // just to cut a "mouth" in sphere
  auto cube = std::make_shared<geo::Cube>();
  cube->transform =  math::translation(0, 0, -1.5) * math::scaling(0.5, 1.4, 1) * math::rotation_y(M_PI / 4);
  cube->material.color = color::BLACK;
  cube->material.cast_shadow = false;

  geo::Cylinder eye_cyl;
  eye_cyl.minimum = 0;
  eye_cyl.maximum = 1;
  eye_cyl.closed = true;
  eye_cyl.material.color = color::Color(1,0,0);
  eye_cyl.transform = math::scaling(0.6 , 1, 1);
  auto eyep = std::make_shared<geo::Cylinder>(eye_cyl);
  wrld.objects.push_back(eyep);

  auto pacman = sphere - cube;
  pacman->transform = math::rotation_y(- M_PI / 4) * math::rotation_z(M_PI / 2);
  //wrld.objects.push_back(pacman);
  
  auto camera = camera::Camera(200, 100, M_PI / 3);
  camera.transform = math::view_transform(math::Point(0, 2.5, -1.5), math::Point(0, 0, 0), math::Vector(0, 1, 0));

  auto image = camera::render_threaded(camera, wrld);
  image.write_ppm("/home/baptiste/csg_pacman.ppm");
  
  return 0;
}
