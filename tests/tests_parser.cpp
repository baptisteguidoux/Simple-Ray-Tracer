#include <string>

#include <gtest/gtest.h>

#include "parser.hpp"

#ifdef RAY_TEST_DIR
#define TEST_DIR RAY_TEST_DIR
#else
#define TEST_DIR "test dir not defined"
#endif


TEST(ParserTest, ParserIgnoresUnrecognizesStatements) {

  parser::ObjParser parser(std::string{TEST_DIR} + "gibberish.obj");
  EXPECT_EQ(parser.ignored_lines, 5);  
}

TEST(ParserTest, ParserProcessVertexDataInput) {

  parser::ObjParser parser(std::string{TEST_DIR} + "four_vertices.obj");
  ASSERT_EQ(parser.vertices.size(), 4);
  EXPECT_EQ(parser.vertices[0], math::Point(-1, 1, 0));
  EXPECT_EQ(parser.vertices[1], math::Point(-1, 0.5, 0));
  EXPECT_EQ(parser.vertices[2], math::Point(1, 0, 0));
  EXPECT_EQ(parser.vertices[3], math::Point(1, 1, 0));
}

TEST(ParserTest, ParserProcessTriangleDataInput) {

  parser::ObjParser parser(std::string{TEST_DIR} + "triangle_faces.obj");
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

  parser::ObjParser parser(std::string{TEST_DIR} + "five_faces_polygon.obj");
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

TEST(ParserTest, NamedGroupInObjFiles) {

  parser::ObjParser parser(std::string{TEST_DIR} + "triangles.obj");
  auto g1 = parser.get_group_by_name("FirstGroup");
  auto g2 = parser.get_group_by_name("SecondGroup");
  auto s1ptr = g1->shapes.at(0);
  auto s2ptr = g2->shapes.at(0);
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

TEST(ParserTest, ConvertObjFileToGroup) {

  parser::ObjParser parser(std::string{TEST_DIR} + "triangles.obj");
  auto g1 = parser.get_group_by_name("FirstGroup");
  auto g2 = parser.get_group_by_name("SecondGroup");  
  auto group = parser.to_group();
  // Check FirstGroup and SecondGroup are present in the group we got from to_group
  auto g1_in_group = std::find_if(group->shapes.begin(), group->shapes.end(),
				  [&](const std::shared_ptr<geo::Shape> shpptr)
				  {return *shpptr == *g1;});
  EXPECT_NE(g1_in_group, group->shapes.end());
  auto g2_in_group = std::find_if(group->shapes.begin(), group->shapes.end(),
				  [&](const std::shared_ptr<geo::Shape> shpptr)
				  {return *shpptr == *g2;});
  EXPECT_NE(g2_in_group, group->shapes.end());  
}

TEST(ParserTest, ParseVertexNormalsInObj) {

  parser::ObjParser parser(std::string{TEST_DIR} + "vertex_normal.obj");
  ASSERT_EQ(parser.normals.size(), 3);
  EXPECT_EQ(parser.normals[0], math::Vector(0, 0, 1));
  EXPECT_EQ(parser.normals[1], math::Vector(0.707, 0, -0.707));
  EXPECT_EQ(parser.normals[2], math::Vector(1, 2, 3));
}

TEST(ParserTest, ParserAssociatesVertexNormalWithFacesInObj) {
 
  parser::ObjParser parser(std::string{TEST_DIR} + "faces_and_vertex_normal.obj");

  ASSERT_EQ(parser.default_group->shapes.size(), 2);
  auto t1 = dynamic_cast<geo::SmoothTriangle*>(parser.default_group->shapes[0].get());
  auto t2 = dynamic_cast<geo::SmoothTriangle*>(parser.default_group->shapes[1].get());
  ASSERT_EQ(parser.vertices.size(), 3);
  EXPECT_EQ(t1->p1, parser.vertices[0]);
  EXPECT_EQ(t1->p2, parser.vertices[1]);  
  EXPECT_EQ(t1->p3, parser.vertices[2]);
  ASSERT_EQ(parser.normals.size(), 3);
  EXPECT_EQ(t1->n1, parser.normals[2]);
  EXPECT_EQ(t1->n2, parser.normals[0]);  
  EXPECT_EQ(t1->n3, parser.normals[1]);
  EXPECT_EQ(*t1, *t2);
}


