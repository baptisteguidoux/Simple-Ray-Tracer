
#include <gtest/gtest.h>

#include "canvas.hpp"


TEST(CanvasTest, CanvasCreation) {
  canvas::Canvas canvas(10, 20);

  ASSERT_EQ(canvas.width, 10);
  ASSERT_EQ(canvas.height, 20);
  // Every pixels of the canvas is initialised to black, ie Color(0, 0, 0)
  for (const auto& pix_row : canvas.pixels)
    for (const auto& pix : pix_row)
      EXPECT_EQ(pix, color::Color(0, 0, 0));
}

TEST(CanvasTest, WritePixel) {
  canvas::Canvas canvas(10, 20);
  color::Color red(1, 0, 0);

  canvas.write_pixel(2, 3, red);
  EXPECT_EQ(canvas.get_pixel(2, 3), red);
  EXPECT_EQ(canvas.get_pixel(1, 1), color::Color(0, 0, 0));
}

TEST(CanvasTest, WritePixelRegion) {
  canvas::Canvas canvas(2, 3);
  canvas.write_region(0, 2, 0, 2, color::Color(0, 0, 1));

  for (const auto& color : canvas.pixels.at(0))
    EXPECT_EQ(color, color::Color(0, 0, 1));
  for (const auto& color : canvas.pixels.at(2))
    EXPECT_EQ(color, color::Color(0, 0, 0));
}

TEST(CanvasTest, ToPPM) {
  canvas::Canvas canvas(5, 3);
  canvas.write_pixel(0, 0, color::Color(1.5, 0, 0));
  canvas.write_pixel(2, 1, color::Color(0, 0.5, 0));
  canvas.write_pixel(4, 2, color::Color(-0.5, 0, 1));

   // Test header, ie first three lines
  auto lines = canvas.to_ppm();
  EXPECT_EQ(lines[0], "P3");
  EXPECT_EQ(lines[1], "5 3");
  EXPECT_EQ(lines[2], "255");
  // Body
  EXPECT_EQ(lines[3], "255 0 0 0 0 0 0 0 0 0 0 0 0 0 0");
  EXPECT_EQ(lines[4], "0 0 0 0 0 0 0 128 0 0 0 0 0 0 0");
  EXPECT_EQ(lines[5], "0 0 0 0 0 0 0 0 0 0 0 0 0 0 255");
  // Lines must not exceed 70 chars long

  canvas::Canvas canvas2(10, 2);
  canvas2.write_region(0, 10, 0, 2, color::Color(1, 0.8, 0.6));
  auto lines2 = canvas2.to_ppm();

  EXPECT_EQ(lines2[3], "255 204 153 255 204 153 255 204 153 255 204 153 255 204 153 255 204");
  EXPECT_EQ(lines2[4], "153 255 204 153 255 204 153 255 204 153 255 204 153");
  EXPECT_EQ(lines2[5], "255 204 153 255 204 153 255 204 153 255 204 153 255 204 153 255 204");
  EXPECT_EQ(lines2[6], "153 255 204 153 255 204 153 255 204 153 255 204 153");
}

TEST(CanvasTest, Concatenation) {

  canvas::Canvas canvas1(4, 2);
  canvas::Canvas canvas2(4, 2);
  canvas::Canvas canvas3(4, 2);

  canvas1.write_region(0, 4, 0, 2, color::Color(1, 0, 0));
  canvas2.write_region(0, 4, 0, 2, color::Color(0, 1, 0));
  canvas3.write_region(0, 4, 0, 2, color::Color(0, 0, 1));  

  canvas::Canvas concatenated_canvas = canvas1 + canvas2 + canvas3;

// Ensure canvas height memeber variable is what we wanted  
  EXPECT_EQ(concatenated_canvas.pixels.size(), 6);
  // Ensure canvas height memeber variable is the actual amount of std::vector<Color> in concatenated_canvas.pixels
  EXPECT_EQ(concatenated_canvas.pixels.size(), concatenated_canvas.height);
  // Same for width
  EXPECT_EQ(concatenated_canvas.pixels.at(0).size(), 4);
  EXPECT_EQ(concatenated_canvas.pixels.at(0).size(), concatenated_canvas.width);
  
  EXPECT_EQ(concatenated_canvas.get_pixel(0, 0), color::Color(1, 0, 0));
  EXPECT_EQ(concatenated_canvas.get_pixel(2, 1), color::Color(1, 0, 0));
  EXPECT_EQ(concatenated_canvas.get_pixel(3, 2), color::Color(0, 1, 0));  
  EXPECT_EQ(concatenated_canvas.get_pixel(3, 3), color::Color(0, 1, 0));
  EXPECT_EQ(concatenated_canvas.get_pixel(0, 5), color::Color(0, 0, 1));
  EXPECT_EQ(concatenated_canvas.get_pixel(3, 5), color::Color(0, 0, 1));
}

