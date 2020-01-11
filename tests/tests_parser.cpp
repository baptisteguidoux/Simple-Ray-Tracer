
#include <gtest/gtest.h>

#include "parser.hpp"

TEST(ParserTest, ParserIgnoresUnrecognizesStatements) {

  parser::ObjParser parser("/home/baptiste/dev/Simple-Ray-Tracer/tests/gibberish.txt");
  EXPECT_EQ(parser.ignored_lines, 5);  
}

