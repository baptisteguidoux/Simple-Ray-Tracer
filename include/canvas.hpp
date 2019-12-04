/* \file canvas.hpp
 */

#ifndef CANVAS_H
#define CANVAS_H


#include <vector>
#include <string>

#include "color.hpp"


/* \namespace canvas
 */
namespace canvas {

  /* \struct Canvas
   */
  struct Canvas {

    int width;
    int height;
    const int maximum_color_value = 255;
    std::vector<std::vector<color::Color>> pixels;
    const static int ppm_line_max_chars = 70;

    /* Canvas constructor
     */    
    Canvas(int w, int h);

    /* \fn color::Color& get_pixel(const int x_pos, const int y_pos)
     * \param x_pos x position
     * \param y_pos y position
     * \return Color of the pixel at the given position in the Canvas
     */    
    color::Color& get_pixel(const int x_pos, const int y_pos);

    /* \fn const color::Color& get_pixel(const int x_pos, const int y_pos) const
     * \param x_pos x position
     * \param y_pos y position
     * \return Color of the pixel at the given position in the Canvas
     */     
    const color::Color& get_pixel(const int x_pos, const int y_pos) const;

    /* \fn void write_pixel(const int x_pos, const int y_pos, const color::Color& color)
     * Set the Color of the point at the given coordinates
     * \param x_pos x position
     * \param y_pos y position
     * \param color the Color to set at the given position
     */    
    void write_pixel(const int x_pos, const int y_pos, const color::Color& color);

    /* \fn void write_region(const int x_start, const int x_end, const int y_start, const int y_end, const color::Color& color)
     * Set the points in the delimited region to the given color
     * \param x_start the x position of the point delimiting the start of the region
     * \param x_end the x position of the point delimiting the end of the region
     * \param y_start the y position of the point delimiting the start of the region
     * \param y_end the y position of the point delimiting the end of the region
     * \param color the Color to set
     */    
    void write_region(const int x_start, const int x_end, const int y_start, const int y_end, const color::Color& color);

    /* \fn void write_ppm(const std::string& ppm_path)
     * Export the Canvas as a ppm file
     * \param ppm_path output path 
     */    
    void write_ppm(const std::string& ppm_path); // CBB: C++ 17 has added a path/filesystem module to the std

    /* \fn std::vector<std::string> to_ppm() const
     * \return the Canvas represented as several strings, each one being a line building the ppm file
     */    
    std::vector<std::string> to_ppm() const;

    /* \fn std::string get_row_ppm_string(const int row) const
     * \param row the position of the row to render as a ppm string
     * \return a ppm string
     */    
    std::string get_row_ppm_string(const int row) const;

    /* \fn std::vector<std::string> get_header() const
     * \return the ppm file required header
     */    
    std::vector<std::string> get_header() const;

  };
  
  /* \fn Canvas operator+(const Canvas& canvas1, const Canvas& canvas2)
   * Add two Canvas, adding each component together
   * \param canvas1 a Canvas
   * \param canvas2 another Canvas
   * \return A new Canvas, sum of the two inputs
   */
  Canvas operator+(const Canvas& canvas1, const Canvas& canvas2);

  /* \namespace utils
   */
  namespace utils {
    
    /* \fn std::vector<std::string> split_in_lines_at_max_length(const std::string& input, const int max_length)
     * Divides a string in substrings, cutting them at the last whitespace before the max_length
     * \param input the string to split in several strings
     * \param max_length the max length of the resulting strings
     * \return the resulting substrings in a vector
     */
    std::vector<std::string> split_in_lines_at_max_length(const std::string& input, const int max_length);
  }

}
  
#endif

