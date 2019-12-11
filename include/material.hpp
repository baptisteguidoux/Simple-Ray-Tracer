/* \file material.hpp
 */

#ifndef MATERIAL_H
#define MATERIAL_H


#include "color.hpp"


/* \namespace material
 */
namespace material {


  /* \struct Material
   */  
  struct Material {

    color::Color color = color::WHITE;
    float ambient = 0.1;
    float diffuse = 0.9;
    float specular = 0.9;
    float shininess = 200;
    
  };

  /* \fn bool operator==(const Material& first, const Material& second);
   * \brief Compares the equality of two Material
   * \param first a Material
   * \param second another Material
   * \return true if the two Material have the same values
   */
  bool operator==(const Material& first, const Material& second);

  /* \fn bool operator!=(const Material& first, const Material& second);
   * \brief Compares the unequality of two Material
   * \param first a Material
   * \param second another Material
   * \return true if the two Material don't have the same values
   */  
  bool operator!=(const Material& first, const Material& second);
  
}

#endif 
