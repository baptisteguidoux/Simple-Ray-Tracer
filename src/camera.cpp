
#include <cmath>
#include <algorithm>
#include <iostream>
#include <thread>
#include <future>

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

  canvas::Canvas Camera::render(world::World& wrld) {

    auto image = canvas::Canvas(hsize, vsize);

    for (int y = 0; y < vsize; y++)
      for (int x = 0; x < hsize; x++) {
  	auto ry = ray_for_pixel(x, y);
  	auto col = wrld.color_at(ry);
  	image.write_pixel(x, y, col);
      }

    return image;
  }
  
  canvas::Canvas render_partial(const Camera& cam,world::World wrld, const int from_y, const int to_y) {

    auto image = canvas::Canvas(cam.hsize, to_y - from_y);
    
    for (int y = from_y; y < to_y; y++)
      for (int x = 0; x < cam.hsize; x++) {
  	auto ry = cam.ray_for_pixel(x, y);
  	auto col = wrld.color_at(ry);
  	image.write_pixel(x, y - from_y, col);
	
      }

    return image;
  }

  canvas::Canvas render_threaded(const Camera& cam, const world::World& wrld) {

    auto available_threads = std::thread::hardware_concurrency();
    if (available_threads == 1) {
      throw std::runtime_error {"Cannot use multithreading, only one thread available"};
    }

    std::cout << "Available threads: " << available_threads << '\n';
    auto working_threads = available_threads - 1;
    std::cout << "Using " << working_threads << " threads\n";

    using Task_type = canvas::Canvas(const Camera&, const world::World&, const int, const int);

    // packaged_task simplify the setting of tasks that use futures to run on threads
    std::vector<std::packaged_task<Task_type>> render_tasks(working_threads);
    for (auto& render_task : render_tasks)
      render_task = std::packaged_task<Task_type> {render_partial};

    // futures avoid returning by reference
    std::vector<std::future<canvas::Canvas>> canvas_futures(working_threads);
    for (size_t i = 0; i < render_tasks.size(); i++)
      canvas_futures[i] = render_tasks[i].get_future();

    // The image to render is divided horizontally, each thread being in charge of one section
    unsigned int thread_render_size = cam.vsize / working_threads;

    std::vector<std::thread> threads;
    for (size_t i = 0; i < render_tasks.size(); i++)
      threads.push_back(std::thread {std::move(render_tasks[i]),
    	                std::ref(cam), std::ref(wrld),
    	                thread_render_size * i, thread_render_size * i + thread_render_size});
    
    // Wait for all the threads to terminate
    std::for_each(threads.begin(), threads.end(), [&](std::thread& thread){thread.join();});

    // concatenate the canvas to obtain final image
    auto result = canvas::Canvas(cam.hsize, 0);
    for (auto& canvas_future : canvas_futures)
      result = result + canvas_future.get();
    
    return result;
  }
  
}

