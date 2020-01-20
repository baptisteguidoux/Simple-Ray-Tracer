#include "tuple.hpp"
#include "matrix.hpp"
#include "world.hpp"
#include "camera.hpp"
#include "light.hpp"
#include "color.hpp"
#include "geo.hpp"
#include "material.hpp"
#include "pattern.hpp"


std::shared_ptr<geo::Shape> build_cube_of_cubes(const uint cubes_per_line, const float space_in_btwn) {

  // space_in_btwn must be > 1

  auto cubes = std::make_shared<geo::Group>();
  
  for (uint x = 0; x < cubes_per_line; x++) {
    for (uint y = 0; y < cubes_per_line; y++) {
      for (uint z = 0; z < cubes_per_line; z++) {
	auto cube = std::make_shared<geo::Cube>();
	cube->transform = math::translation(x + (x * space_in_btwn), y + (y * space_in_btwn), z + (z * space_in_btwn));
	cube->material.color = color::Color(float(y)/cubes_per_line, float(y)/cubes_per_line, float(y)/cubes_per_line);
	cubes->add_child(cube);
      }
    }
  }

  return cubes;
}


int main() {

  auto cam = camera::Camera(200, 100, 0.785);
  cam.transform = math::view_transform(math::Point(0, 1.5, -9), math::Point(0, 1, 0), math::Vector(0, 1, 0));

  auto world = world::World();
  world.light = std::make_shared<light::PointLight>(math::Point(-10, 10, -10), color::Color(1, 1, 1));

  auto floor = std::make_shared<geo::Plane>();
  floor->material.specular = 0;
  floor->material.pattern = std::make_shared<pattern::CheckerPattern>(color::WHITE, color::BLACK);

  auto left_wall = std::make_shared<geo::Plane>();
  left_wall->transform = math::translation(0, 0, 5) * math::rotation_y(-M_PI / 4) * math::rotation_x(M_PI / 2);
  left_wall->material = floor->material;
  left_wall->material.pattern = nullptr;
  left_wall->material.color = color::BLACK;

  auto right_wall = std::make_shared<geo::Plane>();
  right_wall->transform = math::translation(0, 0, 5) * math::rotation_y(M_PI / 4) * math::rotation_x(M_PI / 2);
  right_wall->material = floor->material;
  right_wall->material.pattern = nullptr;
  right_wall->material.color = color::WHITE;  

  world.objects.push_back(floor);
  world.objects.push_back(left_wall);
  world.objects.push_back(right_wall);
  
  auto cubes = build_cube_of_cubes(20, 1.5);

  cubes->transform = math::translation(-1, 0, 0)  * math::rotation_y(M_PI / 3) * math::rotation_x(- M_PI / 4) *  math::scaling(0.05, 0.05, 0.05);
  
  world.objects.push_back(cubes);
  
  auto render = camera::render_threaded(cam, world);
  render.write_ppm("/home/baptiste/cube_of_cubes.ppm");
  

}

