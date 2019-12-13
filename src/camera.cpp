
#include <cmath>

#include "canvas.hpp"
#include "ray.hpp"
#include "camera.hpp"
#include "matrix.hpp"


namespace camera {

  Camera::Camera(const int h, const int v, const float fov) : hsize {h}, vsize {v}, field_of_view {fov} {

    // Determine pixel size
    // determine width of the half canvas
    float half_view = tan(field_of_view / 2.0);
    float aspect_ratio = static_cast<float>(hsize) / static_cast<float>(vsize);
    // wether it's a vertical or horizontal canvas
    if (aspect_ratio >= 1) {
      half_width = half_view;
      half_height = half_view / aspect_ratio;
    } else {
      half_width = half_view * aspect_ratio;
      half_height = half_view;
    }

    // full width of the canvas by the horizontal size of the canvas
    pixel_size = (half_width * 2.0) / hsize;
    
  }
    
  ray::Ray Camera::ray_for_pixel(const int px, const int py) const {

    // Offset from the edge of the canvas to the pixel center
    auto xoffset = (px + 0.5) * pixel_size;
    auto yoffset = (py + 0.5) * pixel_size;

    // the untransformed coordinates of the pixel in world space
    // the camera looks toward -z
    auto world_x = half_width - xoffset;
    auto world_y = half_height - yoffset;

    // using the camera matrix, transform the canvas point and origin
    auto pixel = math::inverse(transform) * math::Point(world_x, world_y, -1);
    auto origin = math::inverse(transform) * math::Point(0, 0, 0);
    // compute ray's direction vector
    auto direction = math::normalize(pixel - origin);

    return ray::Ray(origin, direction);
  }

  canvas::Canvas Camera::render(const world::World& wrld) const {

    auto image = canvas::Canvas(hsize, vsize);

    for (int y = 0; y < vsize; y++)
      for (int x = 0; x < hsize; x++) {
  	auto ry = ray_for_pixel(x, y);
  	auto col = wrld.color_at(ry);
  	image.write_pixel(x, y, col);
      }

    return image;
  }
}

