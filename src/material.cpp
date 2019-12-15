
#include "material.hpp"


namespace material {

  bool operator==(const Material& first, const Material& second) {

    if (first.color == second.color
	&& first.ambient == second.ambient
	&& first.diffuse == second.diffuse
	&& first.specular == second.specular
	&& first.shininess == second.shininess)
      return true;

    return false;
  }
  
  bool operator!=(const Material& first, const Material& second) {

    return !(first == second);
  }
  
}

