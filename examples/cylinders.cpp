
// This file describes the scene illustrated at the start
// of chapter 13, "Cylinders", in "The Ray Tracer Challenge"
// by Jamis Buck <jamis@jamisbuck.org>

#include <cmath>
#include <memory>

#include "camera.hpp"
#include "light.hpp"
#include "geo.hpp"
#include "material.hpp"
#include "world.hpp"
#include "color.hpp"
#include "pattern.hpp"


int main() {

  // camera
  auto cam = camera::Camera(1200, 600, M_PI * 0.1);
  cam.transform = math::view_transform(math::Point(8, 3.5, -9), math::Point(0, 0.3, 0), math::Vector(0, 1, 0));

  // world
  auto world = world::World();
  world.light = light::PointLight(math::Point(1, 6.9, -4.9), color::Color(1, 1, 1));

  // objects
  auto floor = std::make_shared<geo::Plane>();
  floor->material.pattern = std::make_shared<pattern::CheckerPattern>(color::Color(0.5, 0.5, 0.5), color::Color(0.75, 0.75, 0.75));
  floor->material.pattern->transform = math::rotation_y(0.3) * math::scaling(0.25, 0.25, 0.25);
  floor->material.ambient = 0.2;
  floor->material.diffuse = 0.9;
  floor->material.specular = 0;
  world.objects.push_back(floor);

  auto cyl_reflect = std::make_shared<geo::Cylinder>();
  cyl_reflect->minimum = 0;
  cyl_reflect->maximum = 0.75;
  cyl_reflect->closed = true;
  cyl_reflect->transform = math::translation(-1, 0, 1) * math::scaling(0.5, 1, 0.5);
  cyl_reflect->material.color = color::Color(0, 0, 0.6);
  cyl_reflect->material.diffuse = 0.1;
  cyl_reflect->material.specular = 0.9;
  cyl_reflect->material.shininess = 300;
  cyl_reflect->material.reflective = 0.9;
  world.objects.push_back(cyl_reflect);

  auto cyl_concentric1 = std::make_shared<geo::Cylinder>();
  cyl_concentric1->minimum = 0;
  cyl_concentric1->maximum = 0.2;
  cyl_concentric1->closed = false;
  cyl_concentric1->transform = math::translation(1, 0, 0) * math::scaling(0.8, 1, 0.8);
  cyl_concentric1->material.color = color::Color(1, 1, 0.3);
  cyl_concentric1->material.ambient = 0.1;
  cyl_concentric1->material.diffuse = 0.8;
  cyl_concentric1->material.specular = 0.9;
  cyl_concentric1->material.shininess = 300;
  world.objects.push_back(cyl_concentric1);

  auto cyl_concentric2 = std::make_shared<geo::Cylinder>();
  cyl_concentric2->minimum = 0;
  cyl_concentric2->maximum = 0.3;
  cyl_concentric2->closed = false;
  cyl_concentric2->transform = math::translation(1, 0, 0) * math::scaling(0.6, 1, 0.6);
  cyl_concentric2->material.color = color::Color(1, 0.9, 0.4);
  cyl_concentric2->material.ambient = 0.1;
  cyl_concentric2->material.diffuse = 0.8;
  cyl_concentric2->material.specular = 0.9;
  cyl_concentric2->material.shininess = 300;
  world.objects.push_back(cyl_concentric2);

  auto cyl_concentric3 = std::make_shared<geo::Cylinder>();
  cyl_concentric3->minimum = 0;
  cyl_concentric3->maximum = 0.4;
  cyl_concentric3->closed = false;
  cyl_concentric3->transform = math::translation(1, 0, 0) * math::scaling(0.4, 1, 0.4);
  cyl_concentric3->material.color = color::Color(1, 0.8, 0.5);
  cyl_concentric3->material.ambient = 0.1;
  cyl_concentric3->material.diffuse = 0.8;
  cyl_concentric3->material.specular = 0.9;
  cyl_concentric3->material.shininess = 300;
  world.objects.push_back(cyl_concentric3);

  auto cyl_concentric4 = std::make_shared<geo::Cylinder>();
  cyl_concentric4->minimum = 0;
  cyl_concentric4->maximum = 0.5;
  cyl_concentric4->closed = true;
  cyl_concentric4->transform = math::translation(1, 0, 0) * math::scaling(0.2, 1, 0.2);
  cyl_concentric4->material.color = color::Color(1, 0.7, 0.6);
  cyl_concentric4->material.ambient = 0.1;
  cyl_concentric4->material.diffuse = 0.8;
  cyl_concentric4->material.specular = 0.9;
  cyl_concentric4->material.shininess = 300;
  world.objects.push_back(cyl_concentric4);

  auto cyl_dec1 = std::make_shared<geo::Cylinder>();
  cyl_dec1->minimum = 0;
  cyl_dec1->maximum = 0.3;
  cyl_dec1->closed = true;
  cyl_dec1->transform = math::translation(0, 0, -0.75) * math::scaling(0.05, 1, 0.05);
  cyl_dec1->material.color = color::Color(1, 0, 0);
  cyl_dec1->material.ambient = 0.1;
  cyl_dec1->material.diffuse = 0.9;
  cyl_dec1->material.specular = 0.9;
  cyl_dec1->material.shininess = 300;
  world.objects.push_back(cyl_dec1);

  auto cyl_dec2 = std::make_shared<geo::Cylinder>();
  cyl_dec2->minimum = 0;
  cyl_dec2->maximum = 0.3;
  cyl_dec2->closed = true;
  cyl_dec2->transform = math::translation(0, 0, -2.25) * math::rotation_y(-0.15) * math::translation(0, 0, 1.5) * math::scaling(0.05, 1, 0.05);
  cyl_dec2->material.color = color::Color(1, 1, 0);
  cyl_dec2->material.ambient = 0.1;
  cyl_dec2->material.diffuse = 0.9;
  cyl_dec2->material.specular = 0.9;
  cyl_dec2->material.shininess = 300;
  world.objects.push_back(cyl_dec2);

  auto cyl_dec3 = std::make_shared<geo::Cylinder>();
  cyl_dec3->minimum = 0;
  cyl_dec3->maximum = 0.3;
  cyl_dec3->closed = true;
  cyl_dec3->transform = math::translation(0, 0, -2.25) * math::rotation_y(-0.3) * math::translation(0, 0, 1.5) * math::scaling(0.05, 1, 0.05);
  cyl_dec3->material.color = color::Color(0, 1, 0);
  cyl_dec3->material.ambient = 0.1;
  cyl_dec3->material.diffuse = 0.9;
  cyl_dec3->material.specular = 0.9;
  cyl_dec3->material.shininess = 300;
  world.objects.push_back(cyl_dec3);

  auto cyl_dec4 = std::make_shared<geo::Cylinder>();
  cyl_dec4->minimum = 0;
  cyl_dec4->maximum = 0.3;
  cyl_dec4->closed = true;
  cyl_dec4->transform = math::translation(0, 0, -2.25) * math::rotation_y(-0.45) * math::translation(0, 0, 1.5) * math::scaling(0.05, 1, 0.05);
  cyl_dec4->material.color = color::Color(0, 1, 1);
  cyl_dec4->material.ambient = 0.1;
  cyl_dec4->material.diffuse = 0.9;
  cyl_dec4->material.specular = 0.9;
  cyl_dec4->material.shininess = 300;
  world.objects.push_back(cyl_dec4);

  auto cyl_glass = std::make_shared<geo::Cylinder>();
  cyl_glass->minimum = 0.0001;
  cyl_glass->maximum = 0.5;
  cyl_glass->closed = true;
  cyl_glass->transform = math::translation(0, 0, -1.5) * math::scaling(0.33, 1, 0.33);
  cyl_glass->material.color = color::Color(0.25, 0, 0);
  cyl_glass->material.diffuse = 0.1;
  cyl_glass->material.specular = 0.9;
  cyl_glass->material.shininess = 300;
  cyl_glass->material.reflective = 0.9;
  cyl_glass->material.transparency = 0.9;
  cyl_glass->material.refractive_index = 1.5;
  world.objects.push_back(cyl_glass);

  auto render = camera::render_threaded(cam, world);
  render.write_ppm("/home/baptiste/cylinders.ppm");

  return 0;
}

