#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <algorithm>
#include <stdexcept>
#include <numeric>
#include <cmath>

#include "canvas.hpp"
#include "color.hpp"


namespace canvas {

  Canvas::Canvas(int w, int h) : width {w}, height {h} {
    pixels = std::vector<std::vector<color::Color>>(height, std::vector<color::Color>(width, color::Color()));
  }

  color::Color& Canvas::get_pixel(const int x_pos, const int y_pos) {
    return pixels.at(y_pos).at(x_pos);
  }

  const color::Color& Canvas::get_pixel(const int x_pos, const int y_pos) const {
    return pixels.at(y_pos).at(x_pos);
  }

  void Canvas::write_pixel(const int x_pos, const int y_pos, const color::Color& color) {
    get_pixel(x_pos, y_pos) = color;
  }  

  void Canvas::write_region(const int x_start, const int x_end, const int y_start, const int y_end, const color::Color& color) {

    for (int y = y_start; y < y_end; y++)
      for (int x = x_start; x < x_end; x++)
	get_pixel(x, y) = color;
  }

  void Canvas::write_ppm(const std::string& ppm_path) {

    std::vector<std::string> ppm_content = to_ppm();
      
    // Add newline end PPM file
    if (ppm_content.size() > 0)
      ppm_content.at(ppm_content.size() - 1) += "\n";

    std::ofstream of {ppm_path};
    if (! of)
      std::cerr << "cannot open file: " << ppm_path << '\n';

    for (const auto& line : ppm_content)
      of << line + '\n';
  }

  std::vector<std::string> Canvas::to_ppm() const {

    std::vector<std::string> ppm_lines {get_header()};
 
    for (int row = 0; row < height; row++) {
      auto row_string = get_row_ppm_string(row);
      // In case row size > 70
      auto subrowstrings = utils::split_in_lines_at_max_length(row_string, ppm_line_max_chars);
      for (const auto& subrow : subrowstrings)
	ppm_lines.push_back(subrow);
    }
      
    return ppm_lines;
  }

  std::string Canvas::get_row_ppm_string(const int row) const {

    std::string row_str;
      
    for (const auto& color : pixels.at(row)) {
      
      // PPM color is between 0 and maximum_color_value, typically 0 and 255
      // float values are rounded up
      int red = std::max(0, std::min(int(std::round(color.red * maximum_color_value)), maximum_color_value));
      int green = std::max(0, std::min(int(std::round(color.green * maximum_color_value)), maximum_color_value));
      int blue = std::max(0, std::min(int(std::round(color.blue * maximum_color_value)), maximum_color_value));

      row_str += std::string{
	std::to_string(red) + " " +
	  std::to_string(green) + " " +
	  std::to_string(blue) + " "
	  };
    }

    // Remove last white space
    row_str.pop_back();

    return row_str;
  }

  std::vector<std::string> Canvas::get_header() const {

    static const std::string identifier {"P3"}; // Identifier for the the flavor of PPM we using; struct const var?
    std::string dimensions {std::to_string(width) + " " + std::to_string(height)};

    return std::vector<std::string> {identifier, dimensions, std::to_string(maximum_color_value)};
  }

  Canvas operator+(const Canvas& canvas1, const Canvas& canvas2) {

    if (canvas1.width != canvas2.width)
      throw std::runtime_error {"When concatenating two Canvas, their width must be identical"};

    Canvas concatenated_canvas = Canvas(0, 0);

    concatenated_canvas.pixels.insert(concatenated_canvas.pixels.end(), canvas1.pixels.begin(), canvas1.pixels.end());
    concatenated_canvas.pixels.insert(concatenated_canvas.pixels.end(), canvas2.pixels.begin(), canvas2.pixels.end());

    concatenated_canvas.width = canvas1.width;
    concatenated_canvas.height = canvas1.height + canvas2.height;
    
    return concatenated_canvas;
  }

  namespace utils {
    // Split string in substrings (each string size must be < max length)
    std::vector<std::string> split_in_lines_at_max_length(const std::string& input, const int max_length) {

      std::vector<std::string> result;

      size_t start_range_pos = 0;
      size_t end_range_pos = max_length;

      while (end_range_pos < input.size()) {
	end_range_pos = std::min(end_range_pos, input.size());
            
	while (input[end_range_pos] != ' ') // First whitespace at the end of the substring
	  --end_range_pos;

	// 2nd substr arg is not a position but distance from 1st arg
	auto distance =  end_range_pos - start_range_pos;

	result.push_back(input.substr(start_range_pos, distance));
	start_range_pos += distance + 1; // + 1 for whitespace
	// Reset
	end_range_pos = start_range_pos + max_length;
      }
      result.push_back(input.substr(start_range_pos, std::min(end_range_pos, input.size())));
    
      return result;
    }    
    
  }

}

