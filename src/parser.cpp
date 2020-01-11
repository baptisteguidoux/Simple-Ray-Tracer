#include <fstream>
#include <string>

#include "parser.hpp"


namespace parser {

  ObjParser::ObjParser(const std::string_view filepath) {
    
    std::ifstream ifs{std::string(filepath)};
    if (!ifs)
      throw std::runtime_error{"Could not open given file"};

    std::string line;
    while (std::getline(ifs, line)) {
      ignored_lines++;
    }
      
  }
}

