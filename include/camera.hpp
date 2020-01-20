/*! \file camera.hpp
 */

#ifndef CAMERA_H
#define CAMERA_H

#include "canvas.hpp"
#include "ray.hpp"
#include "world.hpp"
#include "matrix.hpp"


/*! \namespace camera
 */
namespace camera {

  /*! \class Camera
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

    /*! \fn ray::Ray ray_for_pixel(const int px, const int py) const
     *  \brief Constructs a Ray that passes through the center of the pixel's coordinates
     *  \param px pixel x position
     *  \param py pixel y position
     *  \return Ray for given pixel
     */
    ray::Ray ray_for_pixel(const int px, const int py) const;

    /*! \fn canvas::Canvas render(world::World& wrld) const
     *  \brief Use the Camera to render an image of the given world
     *  \param wrld World to render
     *  \return the rendered World
     */    
    canvas::Canvas render(world::World& wrld);
 
  };

  /*! \fn canvas::Canvas render_partial(const World& wrld, const int from_y, const int to_y) const
   *  \brief Use the Camera to render a vertical fraction of the final image of the given world
   *  \param cam Camera
   *  \param wrld World to render
   *  \param from_y start of the vertical fraction
   *  \param to_y end of the vertical fraction
   *  \return the partillay rendered World image
   */     
  canvas::Canvas render_partial(const Camera& cam, world::World wrld, const int from_y, const int to_y);
      
  /*! \fn canvas::Canvas render_threaded(const Camera& cam, const world::World& wrld)
   *  \brief Use the Camera to render an image of the given world using several threads
   *  \param cam Camera
   *  \param wrld World to render
   *  \return the rendered World image
   */
  canvas::Canvas render_threaded(const Camera& cam, const world::World& wrld);

}

#endif

