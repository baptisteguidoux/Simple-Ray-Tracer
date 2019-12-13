
/* \file camera.hpp
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "canvas.hpp"
#include "ray.hpp"
#include "world.hpp"
#include "matrix.hpp"


/* \namespace camera
 */
namespace camera {

  /* \class Camera
   */  
  class Camera {
  public:

    int hsize;
    int vsize;
    float field_of_view;
    float pixel_size;
    float half_width;
    float half_height;
    math::Matrix transform = math::IDENTITY_MATRIX;

    /* Camera constructor
     * \param h view height 
     * \pararm v view width
     * \param fov field of view
     */     
    Camera(const int h, const int v, const float fov);

    /* \fn ray::Ray ray_for_pixel(const int px, const int py) const
     * \brief Constructs a Ray that passes through the center of the pixel's coordinates
     * \param px pixel x position
     * \param py pixel y position
     * \return Ray for given pixel
     */
    ray::Ray ray_for_pixel(const int px, const int py) const;

    /* \fn canvas::Canvas render(const world::World& wrld) const
     * \brief Use the Camera to render an image of the given world
     * \param wrld World to render
     * \return the rendered World
     */    
    canvas::Canvas render(const world::World& wrld) const;
    
  };
}

#endif

