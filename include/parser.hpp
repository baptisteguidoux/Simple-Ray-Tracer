#ifndef PARSER_H
#define PARSER_H

#include <string_view>
#include <string>
#include <vector>
#include <regex>
#include <unordered_map>

#include "tuple.hpp"
#include "geo.hpp"


namespace parser {

  class ObjParser {
  public:
    int ignored_lines = 0;
    std::vector<math::Tuple> vertices; /// store the parsed vertices in order
    std::shared_ptr<geo::Group> default_group = std::make_shared<geo::Group>(); /// to receive geometry
    std::unordered_map<std::string, std::shared_ptr<geo::Group>> named_groups;
    std::string last_group_added;

    ObjParser(const std::string_view filepath);

    /*! \fn geo::Group* get_group_by_name(const std::string& name)
     *  \brief Look for a Obj Group by its name
     *  \param name group's name
     *  \return a pointer to the Group
     */ 
    geo::Group* get_group_by_name(const std::string& name);

    /*! \fn std::shared_ptr<geo::Group> to_group() const
     *  \brief Convert the OBJ file as a Group
     *  \return a Group containing all shapes and subgroups
     */
    std::shared_ptr<geo::Group> to_group() const;

  private:
    /*! \fn std::vector<std::shared_ptr<geo::Triangle>> fan_triangulation(const std::vector<int>& vertices_idx) const
     *  \brief Break convex polygons into triangles, through a "fan triangulation"
     *  \param vertices_idx vector of int, representing the position of the vertex in vertices member var
     *  \return a vector of triangles
     */
    std::vector<std::shared_ptr<geo::Triangle>> fan_triangulation(const std::vector<int>& vertices_idx) const;
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

  static const std::regex RE_VERTEX_PATTERN {R"(^v\s(-?\d(?:\.\d+)?)\s(-?\d(?:\.\d+)?)\s(-?\d(?:\.\d+)?))"}; /// a 'v' followed by three int or floats

  static const std::regex RE_FACE_PATTERN {R"(^f(\s\d){3,})"}; /// a 'f' followed by three or more int

  static const std::regex RE_FACE_IDX {R"((\d))"};

  static const std::regex RE_NAMED_GROUP {R"(^g\s(\w+))"};
}

#endif


