#ifndef PARSER_H
#define PARSER_H

#include <string_view>
#include <string>
#include <vector>
#include <regex>

#include "tuple.hpp"
#include "geo.hpp"


namespace parser {

  class ObjParser {
  public:
    int ignored_lines = 0;
    std::vector<math::Tuple> vertices; /// store the parsed vertices in order
    std::shared_ptr<geo::Group> default_group = std::make_shared<geo::Group>(); /// to receive geometry

    ObjParser(const std::string_view filepath);
  };

  /*! \fn double string_to_double(const std::string& s)
   *  \brief convert a string to a double
   *  \param s the string
   *  \return a double
   */ 
  double string_to_double(const std::string& s);

  /*! \fn int string_to_int(const std::string& s)
   *  \brief convert a string to a int
   *  \param s the string
   *  \return a int
   */   
  int string_to_int(const std::string& s);
  
}
#endif

#ifndef PARSER_STATIC_CONSTANT
#define PARSER_STATIC_CONSTANT

namespace parser {

  static const std::regex RE_VERTEX_PATTERN {R"(v\s(-?\d(?:\.\d+)?)\s(-?\d(?:\.\d+)?)\s(-?\d(?:\.\d+)?))"}; /// a 'v' followed by three int or floats

  static const std::regex RE_FACE_PATTERN {R"(f\s(\d)\s(\d)\s(\d))"}; /// a 'f' followed by three int
}

#endif


