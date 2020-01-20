#include <memory>
#include <cmath>

#include "tuple.hpp"
#include "world.hpp"
#include "geo.hpp"
#include "color.hpp"
#include "light.hpp"
#include "camera.hpp"
#include "material.hpp"

int main() {

  auto cam = camera::Camera(400, 200, M_PI/3);
  cam.transform = math::view_transform(math::Point(0, 3.1, -2.5), math::Point(0, 1, 0.4), math::Vector(0, 1, 0));

  auto world = world::World();
  world.light = std::make_shared<light::PointLight>(math::Point(-10, 10, -10), color::Color(1, 1, 1));

  auto floor = std::make_shared<geo::Plane>();
  floor->material.color = color::Color(0.48,0.84,1.00); //light blue
  world.objects.push_back(floor);
  
  auto ice_cream_material = material::Material();
  ice_cream_material.ambient = 0.2;
  ice_cream_material.diffuse = 0.5;
  ice_cream_material.specular = 0.1;
  ice_cream_material.shininess = 50;

  auto ice_cream_cone = std::make_shared<geo::DoubleCone>(); // truncate it to keep a simple cone
  ice_cream_cone->minimum = -1;
  ice_cream_cone->maximum = 0;
  ice_cream_cone->transform = math::translation(0, -0.2, -0) * math::rotation_z(M_PI) * math::scaling(0.6, 2, 0.6);
  ice_cream_cone->material.color = color::Color(1, 179.0/255.0, 102.0/255.0);
  world.objects.push_back(ice_cream_cone);

  auto ice_cream_ball1 = std::make_shared<geo::Sphere>();
  ice_cream_ball1->transform = math::translation(0.2, 1.7, 0.3) * math::scaling(0.4, 0.4, 0.4);
  ice_cream_ball1->material = ice_cream_material;
  ice_cream_ball1->material.color = color::Color(252.0/255, 90.0/255.0, 141.0/255); // strawberry
  world.objects.push_back(ice_cream_ball1);

  auto ice_cream_ball2 = std::make_shared<geo::Sphere>();
  ice_cream_ball2->transform = math::translation(-0.2, 1.7, 0.3) * math::scaling(0.4, 0.4, 0.4);
  ice_cream_ball2->material = ice_cream_material;
  ice_cream_ball2->material.color = color::Color(243.0/255.0, 229.0/255.0, 171.0/255.0); // vanilla
  world.objects.push_back(ice_cream_ball2);

  auto ice_cream_ball3 = std::make_shared<geo::Sphere>();
  ice_cream_ball3->transform = math::translation(0, 1.7, -0.1) * math::scaling(0.4, 0.4, 0.4);
  ice_cream_ball3->material = ice_cream_material;
  ice_cream_ball3->material.color = color::Color(123.0/255.0, 63.0/255.0, 0); //chocolate
  world.objects.push_back(ice_cream_ball3);  
  
  auto render = camera::render_threaded(cam, world);
  render.write_ppm("/home/baptiste/cone_ice_cream.ppm");
  
  return 0;
}

