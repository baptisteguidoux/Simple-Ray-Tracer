#include "color.hpp"
#include "tuple.hpp"

namespace color {

  Color::Color(const float r, const float g, const float b) : red {r}, green {g}, blue {b} {}

  bool operator==(const Color& first, const Color& second) {
    return math::almost_equal(first.red, second.red)
      && math::almost_equal(first.green, second.green)
      && math::almost_equal(first.blue, second.blue);
  }

  bool operator!=(const Color& first, const Color& second) {
    return ! (first == second);
  }

  Color operator+(const Color& first, const Color& second) {
    return Color(first.red + second.red, first.green + second.green, first.blue + second.blue);
  }

  Color operator-(const Color& first, const Color& second) {
    return Color(first.red - second.red, first.green - second.green, first.blue - second.blue);
  }

  Color operator-(const double scalar, const Color& color) {
    return Color(scalar - color.red, scalar - color.green, scalar - color.blue);
  }  

  Color operator*(const Color& first, const Color& second) {
    return Color(first.red * second.red, first.green * second.green, first.blue * second.blue);
  }
  
  Color operator*(const Color& color, const double scalar) {
    return Color(color.red * scalar, color.green * scalar, color.blue * scalar);
  }

  Color operator/(const Color& color, const double scalar) {
    return Color(color.red / scalar, color.green / scalar, color.blue / scalar);
  }

  std::ostream& operator<<(std::ostream& os, const Color& color) {
    os << '(' << color.red << ", " << color.green << ", " << color.blue << ")\n";
    return os;
  }

}
