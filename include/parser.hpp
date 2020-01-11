
#include <string_view>


namespace parser {

  class ObjParser {
  public:
    int ignored_lines = 0;

    ObjParser(const std::string_view filepath);
  };
  
}
