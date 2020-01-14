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
    uint faces_count = 0;
    uint vertex_normal_count = 0;
    while (std::getline(ifs, line)) {
      std::smatch matches;
      if (std::regex_search(line, matches, RE_VERTEX_PATTERN)) {
	double x = string_to_double(matches[1]);
	double y = string_to_double(matches[2]);
	double z = string_to_double(matches[3]);
	vertices.push_back(math::Point(x, y, z));
      } else if (std::regex_search(line, matches, RE_VERTEX_NORMAL_PATTERN)) {
	vertex_normal_count++;
	double x = string_to_double(matches[1]);
	double y = string_to_double(matches[2]);
	double z = string_to_double(matches[3]);
	normals.push_back(math::Vector(x, y, z));
	
      } else if (std::regex_search(line, matches, RE_FACE_COMPLEX_PATTERN)) {
	faces_count++;
	std::vector<int> vertices_idx;
	std::vector<int> normals_idx;
	for (std::sregex_iterator p(line.begin(), line.end(), RE_FACE_COMPLEX_IDX);
	     p != std::sregex_iterator{}; p++) {
	  auto vertex = string_to_int((*p)[1]);
	  // auto vertex_texture = string_to_int((*p)[2]);
	  auto vertex_normal = string_to_int((*p)[3]);
	  vertices_idx.push_back(vertex);
	  normals_idx.push_back(vertex_normal);

	  }

	  // If no vertex normal is 0, and the size of normals_idx == size of vertices_idx, we build some smooth triangles
	  std::vector<std::shared_ptr<geo::Shape>> triangles;
	  if (std::find(normals_idx.begin(), normals_idx.end(), 0) == normals_idx.end()
	      && normals_idx.size() == vertices_idx.size())
	    triangles = fan_smoothtriangulation(vertices_idx, normals_idx);
	  else
	    triangles = fan_triangulation(vertices_idx);

	  for (const auto triangle : triangles) {
	    if (last_group_added != "")
	      get_group_by_name(last_group_added)->add_child(triangle);
	    else
	      default_group->add_child(triangle);	
	}
	
      } else if (std::regex_search(line, matches, RE_FACE_PATTERN)) {
	faces_count++;
	std::vector<int> vertices_idx;
	for (std::sregex_iterator p(line.begin(), line.end(), RE_FACE_IDX);
	     p != std::sregex_iterator{}; p++) 
	  vertices_idx.push_back(string_to_int((*p)[1]));

	auto triangles = fan_triangulation(vertices_idx);
	for (const auto triangle : triangles) {
	  if (last_group_added != "")
	    get_group_by_name(last_group_added)->add_child(triangle);
	  else
	    default_group->add_child(triangle);
	}
	
      } else if (std::regex_search(line, matches, RE_NAMED_GROUP)) {

	auto group_name = matches[1];
	std::cout << group_name << std::endl;
	named_groups[group_name] = std::make_shared<geo::Group>();
	last_group_added = group_name;
      } else {
	ignored_lines++;
      }
    }

    std::cout << "Found " << vertices.size() << " vertices\n";
    std::cout << "Found " << vertex_normal_count << " vertex normals\n";    
    std::cout << "Found " << faces_count << " faces\n";

  }

    std::shared_ptr<geo::Group> ObjParser::get_group_by_name(const std::string& name) {

      return named_groups[name];
    }

  std::shared_ptr<geo::Group> ObjParser::to_group() const {

    for (const auto& group : named_groups)
      default_group->add_child(group.second);

    return default_group;
  }

  std::vector<std::shared_ptr<geo::Shape>> ObjParser::fan_triangulation(const std::vector<int>& vertices_idx) {
    
    std::vector<std::shared_ptr<geo::Shape>> triangles;
    
    for (size_t i = 2; i < vertices_idx.size(); i++) {
      const math::Tuple& a = vertices[vertices_idx[0] - 1]; // obj file vertices index start at 1
      const math::Tuple& b = vertices[vertices_idx[i-1] - 1];
      const math::Tuple& c = vertices[vertices_idx[i] - 1];
      auto triangle = std::make_shared<geo::Triangle>(a, b, c);
      triangles.push_back(triangle);
    }

    return triangles;
  }

  std::vector<std::shared_ptr<geo::Shape>> ObjParser::fan_smoothtriangulation(const std::vector<int>& vertices_idx, const std::vector<int>& normals_idx) {

    std::vector<std::shared_ptr<geo::Shape>> smooth_triangles;
    
    for (size_t i = 2; i < vertices_idx.size(); i++) {

      const math::Tuple p1 = vertices[vertices_idx[0] - 1]; // obj file vertices index start at 1
      const math::Tuple p2 = vertices[vertices_idx[i - 1] - 1];
      const math::Tuple p3 = vertices[vertices_idx[i] - 1];

      const math::Tuple n1 = normals[normals_idx[0] - 1];
      const math::Tuple n2 = normals[normals_idx[i - 1] - 1];
      const math::Tuple n3 = normals[normals_idx[i] - 1];
      auto smooth_triangle = std::make_shared<geo::SmoothTriangle>(p1, p2, p3, n1, n2, n3);
      smooth_triangles.push_back(smooth_triangle);
    }

    return smooth_triangles;    
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

