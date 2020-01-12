#include <string>

#include <gtest/gtest.h>

#include "parser.hpp"

#ifdef RAY_TEST_DIR
#define TEST_DIR RAY_TEST_DIR
#else
#define TEST_DIR "test dir not defined"
#endif


TEST(ParserTest, ParserIgnoresUnrecognizesStatements) {

  parser::ObjParser parser(std::string{TEST_DIR} + "gibberish.txt");
  EXPECT_EQ(parser.ignored_lines, 5);  
}

TEST(ParserTest, ParserProcessVertexDataInput) {

  parser::ObjParser parser(std::string{TEST_DIR} + "four_vertices.txt");
  ASSERT_EQ(parser.vertices.size(), 4);
  EXPECT_EQ(parser.vertices[0], math::Point(-1, 1, 0));
  EXPECT_EQ(parser.vertices[1], math::Point(-1, 0.5, 0));
  EXPECT_EQ(parser.vertices[2], math::Point(1, 0, 0));
  EXPECT_EQ(parser.vertices[3], math::Point(1, 1, 0));
}

TEST(ParserTest, ParserProcessTriangleDataInput) {

  parser::ObjParser parser(std::string{TEST_DIR} + "triangle_faces.txt");
  geo::Group* g = parser.default_group.get();
  auto s1ptr = g->shapes.at(0);
  auto s2ptr = g->shapes.at(1);
  auto t1 = dynamic_cast<const geo::Triangle*>(s1ptr.get());
  auto t2 = dynamic_cast<const geo::Triangle*>(s2ptr.get());  

  ASSERT_EQ(parser.vertices.size(), 4);
  EXPECT_EQ(t1->p1, parser.vertices[0]);
  EXPECT_EQ(t1->p2, parser.vertices[1]);
  EXPECT_EQ(t1->p3, parser.vertices[2]);
  EXPECT_EQ(t2->p1, parser.vertices[0]);
  EXPECT_EQ(t2->p2, parser.vertices[2]);
  EXPECT_EQ(t2->p3, parser.vertices[3]);
}

TEST(ParserTest, ParserProcessTriangulatePolygonalData) {

  parser::ObjParser parser(std::string{TEST_DIR} + "five_faces_polygon.txt");
  geo::Group* g = parser.default_group.get();
  auto s1ptr = g->shapes.at(0);
  auto s2ptr = g->shapes.at(1);
  auto s3ptr = g->shapes.at(2);
  auto t1 = dynamic_cast<const geo::Triangle*>(s1ptr.get());
  auto t2 = dynamic_cast<const geo::Triangle*>(s2ptr.get());
  auto t3 = dynamic_cast<const geo::Triangle*>(s3ptr.get());

  ASSERT_EQ(parser.vertices.size(), 5);
  EXPECT_EQ(t1->p1, parser.vertices[0]);
  EXPECT_EQ(t1->p2, parser.vertices[1]);
  EXPECT_EQ(t1->p3, parser.vertices[2]);
  EXPECT_EQ(t2->p1, parser.vertices[0]);
  EXPECT_EQ(t2->p2, parser.vertices[2]);
  EXPECT_EQ(t2->p3, parser.vertices[3]);
  EXPECT_EQ(t3->p1, parser.vertices[0]);
  EXPECT_EQ(t3->p2, parser.vertices[3]);
  EXPECT_EQ(t3->p3, parser.vertices[4]);
}



