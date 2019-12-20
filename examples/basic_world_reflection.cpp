#include <cmath>
#include <memory>

#include "tuple.hpp"
#include "world.hpp"
#include "geo.hpp"
#include "color.hpp"
#include "light.hpp"
#include "camera.hpp"


int main() {

  auto floor = std::make_shared<geo::Plane>();
  floor->transform = math::scaling(10, 0.01, 10);
  floor->material.color = color::Color(1, 0.9, 0.9);
  floor->material.specular = 0;
  floor->material.color = color::BLACK;
  floor->material.reflective = 0.9;

  // same material as floor
  auto left_wall = std::make_shared<geo::Plane>();
  left_wall->transform = math::translation(0, 0, 5) * math::rotation_y(-M_PI / 4) * math::rotation_x(M_PI / 2);
  left_wall->material = floor->material;

  // right wall is almost identical to left one
  auto right_wall = std::make_shared<geo::Plane>();
  right_wall->transform = math::translation(0, 0, 5) * math::rotation_y(M_PI / 4) * math::rotation_x(M_PI / 2);
  right_wall->material = floor->material;

  // large green sphere in the middle
  auto middle = std::make_shared<geo::Sphere>();
  middle->transform = math::translation(-0.5, 1, 0.5);
  middle->material.color = color::Color(0.1, 1, 0.5);
  middle->material.diffuse = 0.7;
  middle->material.specular = 0.3;

  // smaller green sphere, on the right
  auto right = std::make_shared<geo::Sphere>();
  right->transform = math::translation(1.5, 0.5, -0.5) * math::scaling(0.5, 0.5, 0.5);
  right->material.color = color::Color(0.5, 1, 0.1);
  right->material.diffuse = 0.7;
  right->material.specular = 0.3;

  // smallest yellowish sphere on the left
  auto left =  std::make_shared<geo::Sphere>();
  left->transform = math::translation(-1.5, 0.33, -0.75) * math::scaling(0.33, 0.33, 0.33);
  left->material.color = color::Color(1, 0.8, 0.1);
  left->material.diffuse = 0.7;
  left->material.specular = 0.3;

  auto wrld = world::World();
  // white light source, from above and the left
  wrld.light = light::PointLight(math::Point(-10, 10, -10), color::Color(1, 1, 1));

  wrld.objects.push_back(floor);
  wrld.objects.push_back(left_wall);
  wrld.objects.push_back(right_wall);
  wrld.objects.push_back(middle);
  wrld.objects.push_back(right);
  wrld.objects.push_back(left);  
  
  auto camera = camera::Camera(200, 100, M_PI / 3);
  //auto camera = camera::Camera(1000, 500, M_PI / 3);
  camera.transform = math::view_transform(math::Point(0, 1.5, -5), math::Point(0, 1, 0), math::Vector(0, 1, 0));

  auto image = camera::render_threaded(camera, wrld);
  image.write_ppm("/home/baptiste/basic_world_reflection.ppm");

  return 0;
}

