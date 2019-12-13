
#include <cmath>

#include <gtest/gtest.h>

#include "camera.hpp"
#include "matrix.hpp"
#include "tuple.hpp"


TEST(CameraTest, Constructor) {

  // Constructing a Camera
  int hsize = 160;
  int vsize = 120;
  float field_of_view = M_PI / 2;
  auto cam = camera::Camera(hsize, vsize, field_of_view);
  EXPECT_EQ(cam.hsize, 160);
  EXPECT_EQ(cam.vsize, 120);
  EXPECT_TRUE(math::almost_equal(cam.field_of_view, M_PI / 2));
  EXPECT_EQ(cam.transform, math::IDENTITY_MATRIX);
}

TEST(CameraTest, PixelSize) {

  // The pixel size of a horizontal Canvas
  auto cam = camera::Camera(200, 125, M_PI / 2);
  EXPECT_TRUE(math::almost_equal(cam.pixel_size, 0.01));

  // The pixel size of a vertical Canvas
  cam = camera::Camera(125, 200, M_PI / 2);
  EXPECT_TRUE(math::almost_equal(cam.pixel_size, 0.01));
}

TEST(CameraTest, RayForPixel) {

  // Constructing a ray through the center of the canvas
  auto cam = camera::Camera(201, 101, M_PI / 2);
  auto r = cam.ray_for_pixel(100, 50);
  EXPECT_EQ(r.origin, math::Point(0, 0, 0));
  EXPECT_EQ(r.direction, math::Vector(0, 0, -1));

  // Constructing a ray through a corner of the canvas
  cam = camera::Camera(201, 101, M_PI / 2);
  r = cam.ray_for_pixel(0, 0);
  EXPECT_EQ(r.origin, math::Point(0, 0, 0));
  EXPECT_EQ(r.direction, math::Vector(0.66519, 0.33259, -0.66851));

  // Constructing a ray when the camera is transformed
  cam = camera::Camera(201, 101, M_PI / 2);
  cam.transform = math::rotation_y(M_PI / 4) * math::translation(0, -2, 5);
  r = cam.ray_for_pixel(100, 50);
  EXPECT_EQ(r.origin, math::Point(0, 2, -5)); // the camera's transform describes how the world is moved relative to the camera
  EXPECT_EQ(r.direction, math::Vector(sqrt(2) / 2, 0, - sqrt(2) / 2));
}

TEST(CameraTest, CameraRender) {

  // Rendering a world with a camera
  auto w = world::build_default_world();
  auto cam = camera::Camera(11, 11, M_PI / 2);
  auto from = math::Point(0, 0, -5);
  auto to = math::Point(0, 0, 0);
  auto up = math::Vector(0, 1, 0);
  cam.transform = math::view_transform(from, to, up);
  auto image = cam.render(w);
  EXPECT_EQ(image.get_pixel(5, 5), color::Color(0.38066, 0.47583, 0.2855));
}

TEST(CameraTest, CameraRenderPartial)  {

  // Rendering a world with a camera
  auto w = world::build_default_world();
  auto cam = camera::Camera(11, 11, M_PI / 2);
  auto from = math::Point(0, 0, -5);
  auto to = math::Point(0, 0, 0);
  auto up = math::Vector(0, 1, 0);
  cam.transform = math::view_transform(from, to, up);
  auto image = render_partial(cam, w, 5, 11);
  EXPECT_EQ(image.get_pixel(5, 0), color::Color(0.38066, 0.47583, 0.2855));

}

TEST(CameraTest, CameraRenderThreaded) {

  // Rendering a world with a camera
  auto w = world::build_default_world();
  auto cam = camera::Camera(11, 11, M_PI / 2);
  auto from = math::Point(0, 0, -5);
  auto to = math::Point(0, 0, 0);
  auto up = math::Vector(0, 1, 0);
  cam.transform = math::view_transform(from, to, up);
  auto image = camera::render_threaded(cam, w);
  EXPECT_EQ(image.get_pixel(5, 5), color::Color(0.38066, 0.47583, 0.2855));
}

