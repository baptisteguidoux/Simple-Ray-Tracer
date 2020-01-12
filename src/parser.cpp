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
	int a = string_to_int(matches[1]);
	int b = string_to_int(matches[2]);
	int c = string_to_int(matches[3]);
	auto triangle = std::make_shared<geo::Triangle>
	  (vertices[a-1], vertices[b-1], vertices[c-1]);
	default_group->add_child(triangle.get());
	
      }else {
	ignored_lines++;
      }
    }
      
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

