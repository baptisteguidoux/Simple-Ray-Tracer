/*! \file color.hpp
 */

#ifndef COLOR_H
#define COLOR_H

#include <iostream>

/*! \namespace color
 */
namespace color {

  /*! \struct Color
   *  \brief Just red, green and blue
   */
  struct Color {
    
    float red;
    float green;
    float blue;

    /* Color constructor
     */
    Color(const float r, const float g, const float b);

    Color() = default;
  };

  /*! \fn bool operator==(const Color& first, const Color& second)
   *  \brief Compare equality of two Colors
   *  \param first a Color
   *  \param second another Color
   *  \return true if the two Colors are equal
   */
  bool operator==(const Color& first, const Color& second);

  /*! \fn bool operator!=(const Color& first, const Color& second)
   *  \brief Compare unequality of two Colors
   *  \param first a Color
   *  \param second another Color
   *  \return true if the two Colors are not equal
   */
  bool operator!=(const Color& first, const Color& second);

  /*! \fn operator+(const Color& first, const Color& second)
   *  \brief Sum of two Colors
   *  \param first a Color
   *  \param second another Color
   *  \return A new Color, sum of the two inputs
   */  
  Color operator+(const Color& first, const Color& second);

  /*! \fn operator-(const Color& first, const Color& second)
   *  \brief Difference of two Colors
   *  \param first a Color
   *  \param second another Color
   *  \return A new Color, difference of the two inputs
   */  
  Color operator-(const Color& first, const Color& second);

  /*! \fn operator-(const double scalar, const Color& color)
   *  \brief Substract a scalar with a Color
   *  \param scalar
   *  \param color a Color
   *  \return A Color where each component is the difference between the scalar and the color component
   */    
  Color operator-(const double scalar, const Color& color);
  
  /*! \fn Color operator*(const Color& first, const Color& second)
   *  \brief Multiply a Color with another Color
   *  \param first a Color
   *  \param second another Color
   *  \return A new Color where each component is the product of the two same components in the inputs
   */  
  Color operator*(const Color& first, const Color& second);
  
  /*! \fn Color operator*(const Color& color, const double scalar)
   *  \brief Multiply a Color with a scalar
   *  \param color a Color
   *  \param scalar a scalar value
   *  \return A new Color, each element of the color multiplied by the scalar
   */  
  Color operator*(const Color& color, const double scalar);

  /*! \fn Color operator/(const Color& color, const double scalar)
   *  \brief Divide a Color by a scalar
   *  \param color a Color
   *  \param scalar a scalar value
   *  \return A new Color, each element of the color divided by the scalar
   */    
  Color operator/(const Color& color, const double scalar);

    /*! \fn std::ostream& operator<<(std::ostream& os, const Color& color)
   *  \brief Format and output a Color
   *  \param os output stream
   *  \param color a Color
   *  \return an output stream
   */
  std::ostream& operator<<(std::ostream& os, const Color& color);

}

#endif


#ifndef COLOR_STATIC_CONSTANTS_H
#define COLOR_STATIC_CONSTANTS_H

namespace color {

  static const Color BLACK(0, 0, 0);
  static const Color WHITE(1, 1, 1);

}


#endif

