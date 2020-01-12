#include <fstream>
#include <string>
#include <regex>
#include <sstream>

#include <iostream>

#include "parser.hpp"


namespace parser {

  ObjParser::ObjParser(const std::string_view filepath) {
    
    std::ifstream ifs{std::string(filepath)};
    if (!ifs)
      throw std::runtime_error{"Could not open file: " + std::string(filepath)};

    std::string line;
    while (std::getline(ifs, line)) {
      std::smatch matches;
      if (std::regex_search(line, matches, RE_VERTEX_PATTERN)) {
	double x = string_to_double(matches[1]);
	double y = string_to_double(matches[2]);
	double z = string_to_double(matches[3]);
	vertices.push_back(math::Point(x, y, z));
      } else if (std::regex_search(line, matches, RE_FACE_PATTERN)) {
	
	std::vector<int> vertices_idx;
	for (std::sregex_iterator p(line.begin(), line.end(), RE_FACE_IDX);
	     p != std::sregex_iterator{}; p++) 
	  vertices_idx.push_back(string_to_int((*p)[1]));

	auto triangles = fan_triangulation(vertices_idx);
	for (const auto triangle : triangles)
	  default_group->add_child(triangle.get());
	
      }else {
	ignored_lines++;
      }
    }
      
  }

  std::vector<std::shared_ptr<geo::Triangle>> ObjParser::fan_triangulation(const std::vector<int>& vertices_idx) const {
    
    std::vector<std::shared_ptr<geo::Triangle>> triangles;
    
    for (int i = 2; i < vertices_idx.size(); i++) {
      const math::Tuple& a = vertices[vertices_idx[0] - 1]; // obj file vertices index start at 1
      const math::Tuple& b = vertices[vertices_idx[i-1] - 1];
      const math::Tuple& c = vertices[vertices_idx[i] - 1];
      auto triangle = std::make_shared<geo::Triangle>(a, b, c);
      triangles.push_back(triangle);
    }

    return triangles;
  }

  double string_to_double(const std::string& s) {
  // from: https://stackoverflow.com/questions/392981/how-can-i-convert-string-to-double-in-c
    std::istringstream i(s);
    double x;
    if (!(i >> x))
      return 0;
    return x;    
  }

  int string_to_int(const std::string& s) {
  // from: https://stackoverflow.com/questions/392981/how-can-i-convert-string-to-double-in-c
    std::istringstream i(s);
    int x;
    if (!(i >> x))
      return 0;
    return x;
  }  
}

