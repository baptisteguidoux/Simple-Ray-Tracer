// ======================================================

// This file describes the scene illustrated at the start
// of chapter 12, "Cubes", in "The Ray Tracer Challenge"
// by Jamis Buck <jamis@jamisbuck.org>
// ======================================================

#include <memory>

#include "camera.hpp"
#include "tuple.hpp"
#include "matrix.hpp"
#include "world.hpp"
#include "light.hpp"
#include "color.hpp"
#include "geo.hpp"
#include "material.hpp"
#include "pattern.hpp"


int main() {

  auto cam = camera::Camera(1200, 600, 0.785);
  cam.transform = math::view_transform(math::Point(8, 6, -8), math::Point(0, 3, 0), math::Vector(0, 1, 0));

  auto world = world::World();
  world.light = light::PointLight(math::Point(0, 6.9, -5), color::Color(1, 1, 0.9));

  // floor and ceiling at the same time
  auto floor = std::make_shared<geo::Cube>();
  floor->transform = math::scaling(20, 7, 20) * math::translation(0, 1, 0);
  floor->material.pattern = std::make_shared<pattern::CheckerPattern>(color::BLACK, color::Color(0.25, 0.25, 0.25));
  floor->material.pattern->transform = math::scaling(0.07, 0.07, 0.07);
  floor->material.ambient = 0.25;
  floor->material.diffuse = 0.7;
  floor->material.specular = 0.9;
  floor->material.shininess = 300;
  floor->material.reflective = 0.1;
  world.objects.push_back(floor);

  auto walls = std::make_shared<geo::Cube>();
  walls->transform = math::scaling(10, 10, 10);
  walls->material.pattern = std::make_shared<pattern::CheckerPattern>(color::Color(0.4863, 0.3765, 0.2941), color::Color(0.3725, 0.2902, 0.2275));
  walls->material.pattern->transform = math::scaling(0.05, 20, 0.05);
  walls->material.ambient = 0.1;
  walls->material.diffuse = 0.7;
  walls->material.specular = 0.9;
  walls->material.shininess = 300;
  walls->material.reflective = 0.1;
  world.objects.push_back(walls);

  auto table_top = std::make_shared<geo::Cube>();
  table_top->transform = math::translation(0, 3.1, 0) * math::scaling(3, 0.1, 2);
  table_top->material.pattern = std::make_shared<pattern::StripePattern>(color::Color(0.5529, 0.4235, 0.3255), color::Color(0.6588, 0.5098, 0.4000));
  table_top->material.pattern->transform = math::scaling(0.05, 0.05, 0.05) * math::rotation_y(0.1);
  table_top->material.ambient = 0.1;
  table_top->material.diffuse = 0.7;
  table_top->material.specular = 0.9;
  table_top->material.shininess = 300;
  table_top->material.reflective = 0.2;
  world.objects.push_back(table_top);

  auto leg1 = std::make_shared<geo::Cube>();
  leg1->transform = math::translation(2.7, 1.5, -1.7) * math::scaling(0.1, 1.5, 0.1);
  leg1->material.color = color::Color(0.5529, 0.4235, 0.3255);
  leg1->material.ambient = 0.2;
  leg1->material.diffuse = 0.7;
  world.objects.push_back(leg1);

  auto leg2 = std::make_shared<geo::Cube>();
  leg2->transform = math::translation(2.7, 1.5, 1.7) * math::scaling(0.1, 1.5, 0.1);
  leg2->material.color = color::Color(0.5529, 0.4235, 0.3255);
  leg2->material.ambient = 0.2;
  leg2->material.diffuse = 0.7;
  world.objects.push_back(leg2);

  auto leg3 = std::make_shared<geo::Cube>();
  leg3->transform = math::translation(-2.7, 1.5, -1.7) * math::scaling(0.1, 1.5, 0.1);
  leg3->material.color = color::Color(0.5529, 0.4235, 0.3255);
  leg3->material.ambient = 0.2;
  leg3->material.diffuse = 0.7;
  world.objects.push_back(leg3);

  auto leg4 = std::make_shared<geo::Cube>();
  leg4->transform = math::translation(-2.7, 1.5, 1.7) * math::scaling(0.1, 1.5, 0.1);
  leg4->material.color = color::Color(0.5529, 0.4235, 0.3255);
  leg4->material.ambient = 0.2;
  leg4->material.diffuse = 0.7;
  world.objects.push_back(leg4);

  auto glass_cube = std::make_shared<geo::Cube>();
  glass_cube->transform = math::translation(0, 3.45001, 0) * math::rotation_y(0.2) * math::scaling(0.25, 0.25, 0.25);
  glass_cube->material.cast_shadow = false;
  glass_cube->material.color = color::Color(1, 1, 0.8);
  glass_cube->material.ambient = 0;
  glass_cube->material.diffuse = 0.3;
  glass_cube->material.specular = 0.9;
  glass_cube->material.shininess = 300;
  glass_cube->material.reflective = 0.7;
  glass_cube->material.refractive_index = 1.5;
  world.objects.push_back(glass_cube);

  auto little_cube1 = std::make_shared<geo::Cube>();
  little_cube1->transform = math::translation(1, 3.35, -0.9) * math::rotation_y(-0.4) * math::scaling(0.15, 0.15, 0.15);
  little_cube1->material.color = color::Color(1, 0.5, 0.5);
  little_cube1->material.reflective = 0.6;
  little_cube1->material.diffuse = 0.4;
  world.objects.push_back(little_cube1);

  auto little_cube2 = std::make_shared<geo::Cube>();
  little_cube2->transform = math::translation(-1.5, 3.27, 0.3) * math::rotation_y(0.4) * math::scaling(0.15, 0.07, 0.15);
  little_cube2->material.color = color::Color(1, 1, 0.5);
  world.objects.push_back(little_cube2);

  auto little_cube3 = std::make_shared<geo::Cube>();
  little_cube3->transform = math::translation(0, 3.25, 1) * math::rotation_y(0.4) * math::scaling(0.2, 0.05, 0.05);
  little_cube3->material.color = color::Color(0.5, 1, 0.5);
  world.objects.push_back(little_cube3);

  auto little_cube4 = std::make_shared<geo::Cube>();
  little_cube4->transform = math::translation(-0.6, 3.4, -1) * math::rotation_y(0.8) * math::scaling(0.05, 0.2, 0.05);
  little_cube4->material.color = color::Color(0.5, 0.5, 1);
  world.objects.push_back(little_cube4);

  auto little_cube5 = std::make_shared<geo::Cube>();
  little_cube5->transform = math::translation(2, 3.4, 1) * math::rotation_y(0.8) * math::scaling(0.05, 0.2, 0.05);
  little_cube5->material.color = color::Color(0.5, 1, 1);
  world.objects.push_back(little_cube5);

  auto frame1 = std::make_shared<geo::Cube>();
  frame1->transform = math::translation(-10, 4, 1) * math::scaling(0.05, 1, 1);
  frame1->material.color = color::Color(0.7098, 0.2471, 0.2196);
  frame1->material.diffuse = 0.6;
  world.objects.push_back(frame1);

  auto frame2 = std::make_shared<geo::Cube>();
  frame2->transform = math::translation(-10, 3.4, 2.7) * math::scaling(0.05, 0.4, 0.4);
  frame2->material.color = color::Color(0.2667, 0.2706, 0.6902);
  frame2->material.diffuse = 0.6;
  world.objects.push_back(frame2);

  auto frame3 = std::make_shared<geo::Cube>();
  frame3->transform = math::translation(-10, 4.6, 2.7) * math::scaling(0.05, 0.4, 0.4);
  frame3->material.color = color::Color(0.3098, 0.5961, 0.3098);
  frame3->material.diffuse = 0.6;
  world.objects.push_back(frame3);

  auto mirror_frame = std::make_shared<geo::Cube>();
  mirror_frame->transform = math::translation(-2, 3.5, 9.95) * math::scaling(5, 1.5, 0.05);
  mirror_frame->material.color = color::Color(0.3882, 0.2627, 0.1882);
  mirror_frame->material.diffuse = 0.7;
  world.objects.push_back(mirror_frame);

  auto mirror = std::make_shared<geo::Cube>();
  mirror->transform = math::translation(-2, 3.5, 9.95) * math::scaling(4.8, 1.4, 0.06);
  mirror->material.color = color::BLACK;
  mirror->material.ambient = 0;
  mirror->material.diffuse = 0;
  mirror->material.specular = 1;
  mirror->material.shininess = 300;
  mirror->material.reflective = 1;
  world.objects.push_back(mirror);

  auto render = camera::render_threaded(cam, world);
  render.write_ppm("/home/baptiste/cube.ppm");

  return 0;
}

