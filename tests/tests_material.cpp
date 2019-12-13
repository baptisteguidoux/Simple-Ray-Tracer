
#include <gtest/gtest.h>

#include "tuple.hpp"
#include "material.hpp"


TEST(MaterialTest, Constructor) {

  // Default material
  auto m = material::Material();

  EXPECT_EQ(m.color, color::Color(1, 1, 1));
  EXPECT_TRUE(math::almost_equal(m.ambient, 0.1));
  EXPECT_TRUE(math::almost_equal(m.diffuse, 0.9));
  EXPECT_TRUE(math::almost_equal(m.specular, 0.9));
  EXPECT_TRUE(math::almost_equal(m.shininess, 200));
  EXPECT_TRUE(m.cast_shadow);
}

TEST(MaterialTest, EqualityComparison) {


  // Default material
  auto m = material::Material();
  auto m2 = material::Material();
  auto m3 = material::Material();
  m3.ambient = 0.2;
  
  EXPECT_EQ(m, m2);
  EXPECT_NE(m, m3);
}

