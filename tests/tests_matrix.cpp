
#include <vector>
#include <cmath>

#include <gtest/gtest.h>

#include "tuple.hpp"
#include "matrix.hpp"


TEST(MatrixTest, Creation) {

  // Matrices will be created from a container of containers of float/double
  // One vector of float == 1 one row
  math::Matrix matrix (std::vector<math::Row>{
      math::Row {1, 2, 3, 4},
      math::Row {5.5, 6.5, 7.5, 8.5},
      math::Row {9, 10, 11, 12},
      math::Row {13.5, 14.5, 15.5, 16.5},
	}
    );

  EXPECT_EQ(matrix(0, 0), 1.0);
  EXPECT_EQ(matrix(0, 3), 4.0);
  EXPECT_EQ(matrix(1, 0), 5.5);
  EXPECT_EQ(matrix(1, 2), 7.5);
  EXPECT_EQ(matrix(2, 2), 11.0);
  EXPECT_EQ(matrix(3, 0), 13.5);
  EXPECT_EQ(matrix(3, 2), 15.5);

  // Not only 4*4 matrices, not only float
  math::Matrix matrix2 (std::vector<math::Row> {
      math::Row {-3.0, 5.0},
      math::Row {1.0, -2.0},
    }
  );

  EXPECT_EQ(matrix2(0, 0), -3.0);
  EXPECT_EQ(matrix2(0, 1), 5.0);
  EXPECT_EQ(matrix2(1, 0), 1.0);
  EXPECT_EQ(matrix2(1, 1), -2.0);

  math::Matrix matrix3 (std::vector<math::Row> {
      math::Row {-3.0, 5.0, 0.0},
      math::Row {1.0, -2.0, 7.0},
      math::Row {0.0, 1.0, 1.0},
    }
  );

  EXPECT_EQ(matrix3(0, 0), -3.0);
  EXPECT_EQ(matrix3(1, 1), -2.0);
  EXPECT_EQ(matrix3(2, 2), 1.0);
}

TEST(MatrixTest, Dimensions) {

  math::Matrix matrix32 (std::vector<math::Row> {
      {math::Row {1.0, 2.0}},
      {math::Row {1.0, 2.0}},
      {math::Row {1.0, 2.0}},
  });

  ASSERT_EQ(matrix32.height, 3);
  ASSERT_EQ(matrix32.width, 2);
}

TEST(MatrixTest, Equality) {

  math::Matrix matrix1 (std::vector<math::Row> {
    math::Row {1, 2, 3, 4},
    math::Row {5, 6, 7, 8},
    math::Row {9, 8, 7, 6},
    math::Row {5, 4, 3, 2},
      }
  );

  math::Matrix matrix2 (std::vector<math::Row> {
    math::Row{1, 2, 3, 4},
    math::Row{5, 6, 7, 8},
    math::Row{9, 8, 7, 6},
    math::Row{5, 4, 3, 2},
      }
  );

  ASSERT_EQ(matrix1, matrix2);

  math::Matrix matrix3 (std::vector<math::Row> {
      math::Row {2, 3, 4, 5},
      math::Row {6, 7, 8, 9},
      math::Row {8, 7, 6, 5},
      math::Row {4, 3, 2, 1},
	}
    );

  ASSERT_NE(matrix1, matrix3);
}

TEST(MatrixTest, CopyConstructor) {
  
  math::Matrix matrix (std::vector<math::Row> {
      math::Row {2, 3, 4, 5},
      math::Row {6, 7, 8, 9},
      math::Row {8, 7, 6, 5},
      math::Row {4, 3, 2, 1},
	}
    );

  math::Matrix matrix_copy(matrix);

  ASSERT_EQ(matrix, matrix_copy);
}


TEST(MatrixTest, MatrixMultiplication) {

  math::Matrix matrix1 (std::vector<math::Row> {
    math::Row {1, 2, 3, 4},
    math::Row {5, 6, 7, 8},
    math::Row {9, 8, 7, 6},
    math::Row {5, 4, 3, 2},
      }
  );
  
  math::Matrix matrix2 (std::vector<math::Row> {
      math::Row {-2, 1, 2, 3},
      math::Row {3, 2, 1, -1},
      math::Row {4, 3, 6, 5},
      math::Row {1, 2, 7, 8},
	}
    );

  math::Matrix multres (std::vector<math::Row> {
      math::Row {20, 22, 50, 48},
      math::Row {44, 54, 114, 108},
      math::Row {40, 58, 110, 102},
      math::Row {16, 26, 46, 42},
	}
    );
  
  ASSERT_EQ((matrix1 * matrix2), multres);

  //Must work when Matrix A. width != Matrix A. height, but Matrix A. width == Matrix B height
  math::Matrix matrix3 (std::vector<math::Row> {
      math::Row {1, 2},
      math::Row {3, 4},
      math::Row {5, 6},	
	}
    );

  math::Matrix matrix4 (std::vector<math::Row> {
      math::Row {1, 3, 5},
      math::Row {2, 4, 6},
	}
    );
  
  math::Matrix multres2 (std::vector<math::Row> {
      math::Row {5, 11, 17},
      math::Row {11, 25, 39},
      math::Row {17, 39, 61}
        }
    );
  
  ASSERT_EQ((matrix3 * matrix4).height, matrix3.height);
  ASSERT_EQ((matrix3 * matrix4).width, matrix4.width);
  ASSERT_EQ((matrix3 * matrix4), multres2);
}

TEST(MatrixTest, TupleMultiplication) {

  math::Matrix matrix (std::vector<math::Row> {
      math::Row {1, 2, 3, 4},
      math::Row {2, 4, 4, 2},
      math::Row {8, 6, 4, 1},
      math::Row {0, 0, 0, 1},
	}
    );

  math::Tuple tuple (1.0, 2.0, 3.0, 1.0);

  ASSERT_EQ((matrix * tuple), math::Tuple(18.0, 24.0, 33.0, 1.0));
}

TEST(MatrixTest, ScalarDivision) {

  math::Matrix matrix (std::vector<math::Row> {
      math::Row {4, 4, 4, 4},
      math::Row {4, 4, 4, 4},
      math::Row {4, 4, 4, 4},
      math::Row {4, 4, 4, 4},	
	}
    );

  math::Matrix result (std::vector<math::Row> {
      math::Row {2, 2, 2, 2},
      math::Row {2, 2, 2, 2},
      math::Row {2, 2, 2, 2},
      math::Row {2, 2, 2, 2},	
	}
    );

  EXPECT_EQ((matrix / 2.0), result);
}

TEST(MatrixTest, IdentityMatrix) {
  /* Like the value `1` is the multiplicative identity,
     the identity matrix, when multipled by another matrix or tuple,
     returns the same matrix or tuple.
     It is used as the default transformation for any object in a scene.
   */

  math::Matrix matrix (std::vector<math::Row> {
      math::Row {0, 1, 2, 4},
      math::Row {1, 2, 4, 8},
      math::Row {2, 4, 8, 16},
      math::Row {4, 8, 16, 32},
	}
    );

  ASSERT_EQ((matrix * math::IDENTITY_MATRIX), matrix);

  math::Tuple tuple(1.0, 2.0, 3.0, 4.0);

  ASSERT_EQ((math::IDENTITY_MATRIX * tuple), tuple);
}

TEST(MatrixTest, Transposition) {

  math::Matrix matrix1 (std::vector<math::Row> {
      math::Row {0, 9, 3, 0},
      math::Row {9, 8, 0, 8},
      math::Row {1, 8, 5, 3},
      math::Row {0, 0, 5, 8},
	}
    );

  math::Matrix matrix2 (std::vector<math::Row> {
      math::Row {0, 9, 1, 0},
      math::Row {9, 8, 8, 0},
      math::Row {3, 0, 5, 5},
      math::Row {0, 8, 3, 8},
	}
    );

  ASSERT_EQ(transpose(matrix1), matrix2);

  // Also works with Matrixes having unequal width and height
  math::Matrix matrix3 (std::vector<math::Row> {
      math::Row {1, 2},
      math::Row {3, 4},
      math::Row {5, 6},	
	}
    );

  math::Matrix matrix4 (std::vector<math::Row> {
      math::Row {1, 3, 5},
      math::Row {2, 4, 6},
	}
    );
    
  ASSERT_EQ(transpose(matrix3), matrix4);
}

TEST(MatrixTest, 2x2Determinant) {

  math::Matrix matrix (std::vector<math::Row> {
      math::Row {1, 5},
      math::Row {-3, 2},
	}
    );

  ASSERT_EQ(determinant(matrix), 17);
}

TEST(MatrixTest, SubMatrix) {

  math::Matrix matrix1 (std::vector<math::Row> {
      math::Row {1, 5, 0},
      math::Row {-3, 2, 7},
      math::Row {0, 6, -3},	  
	}
    );

  math::Matrix submatrix1 (std::vector<math::Row> {
      math::Row {-3, 2},
      math::Row {0, 6,},      
	}
    );

  EXPECT_EQ(submatrix(matrix1, 0, 2), submatrix1);

  math::Matrix matrix2 ( std::vector<math::Row> {
    math::Row {-6, 1, 1, 6},
    math::Row {-8, 5, 8, 6},
    math::Row {-1, 0, 8, 2},
    math::Row {-7, 1, -1, 1},
	}
    );

  math::Matrix submatrix2 ( std::vector<math::Row> {
    math::Row {-6, 1, 6},
    math::Row {-8, 8, 6},
    math::Row {-7, -1, 1},
	}
  );

  EXPECT_EQ(submatrix(matrix2, 2, 1), submatrix2);

}

TEST(MatrixTest, Minor) {

  math::Matrix matrix (std::vector<math::Row> {
      math::Row {3, 5, 0},
      math::Row {2, -1, -7},
      math::Row {6, -1, 5},	
	}
    );

  EXPECT_EQ(determinant(submatrix(matrix, 1, 0)), 25);
  EXPECT_EQ(minor(matrix, 1, 0), 25);
}

TEST(MatrixTest, Cofactor) {

  math::Matrix matrix (std::vector<math::Row> {
      math::Row {3, 5, 0},
      math::Row {2, -1, -7},
      math::Row {6, -1, 5},	
	}
    );

  EXPECT_EQ(minor(matrix, 0, 0), -12);
  EXPECT_EQ(cofactor(matrix, 0, 0), -12);
  EXPECT_EQ(minor(matrix, 1, 0), 25);
  EXPECT_EQ(cofactor(matrix, 1, 0), -25);
}

TEST(MatrixTest, 3x3Determinant) {
  
  math::Matrix matrix (std::vector<math::Row> {
      math::Row {1, 2, 6},
      math::Row {-5, 8, -4},
      math::Row {2, 6, 4},	
	}
    );

  EXPECT_EQ(cofactor(matrix, 0, 0), 56);
  EXPECT_EQ(cofactor(matrix, 0, 1), 12);
  EXPECT_EQ(cofactor(matrix, 0, 2), -46);
  EXPECT_EQ(determinant(matrix), -196); 
}

TEST(MatrixTest, 4x4Determinant) {

  math::Matrix matrix (std::vector<math::Row> {
      math::Row {-2, -8, 3, 5},
      math::Row {-3, 1, 7, 3},
      math::Row {1, 2, -9, 6},
      math::Row {-6, 7, 7, -9},
	}
    );

  EXPECT_EQ(cofactor(matrix, 0, 0), 690);
  EXPECT_EQ(cofactor(matrix, 0, 1), 447);
  EXPECT_EQ(cofactor(matrix, 0, 2), 210);
  EXPECT_EQ(cofactor(matrix, 0, 3), 51);
  EXPECT_EQ(determinant(matrix), -4071);
}

TEST(MatrixTest, IsInvertible) {

  math::Matrix matrix1 (std::vector<math::Row> {
      math::Row {6, 4, 4, 4},
      math::Row {5, 5, 7, 6},
      math::Row {4, -9, 3, -7},
      math::Row {9, 1, 7, -6},
	}
    );

  EXPECT_EQ(determinant(matrix1), -2120);
  EXPECT_TRUE(is_invertible(matrix1));

  math::Matrix matrix2 (std::vector<math::Row> {
      math::Row {-4, 2, -2, -3},
      math::Row {9, 6, 2, 6},
      math::Row {0, -5, 1, -5},
      math::Row {0, 0, 0, 0},
	}
    );

  EXPECT_EQ(determinant(matrix2), 0);
  EXPECT_FALSE(is_invertible(matrix2));
}

TEST(MatrixTest, Inverse) {

  math::Matrix A (std::vector<math::Row> {
      math::Row {-5, 2, 6, -8},
      math::Row {1, -5, 1, 8},
      math::Row {7, 7, -6, -7},
      math::Row {1, -3, 7, 4},
  	}
    );

  math::Matrix B = math::inverse(A);


  EXPECT_EQ(determinant(A), 532);
  EXPECT_EQ(cofactor(A, 2, 3), -160);
  EXPECT_TRUE(math::almost_equal(B(3, 2), (-160.0 / 532.0)));
  EXPECT_EQ(cofactor(A, 3, 2), 105);
  EXPECT_TRUE(math::almost_equal(B(2, 3), (105.0 / 532.0)));

  // What should be the result of inverse(A)
  math::Matrix C (std::vector<math::Row> {
      math::Row {0.21805, 0.45113, 0.24060, -0.04511},
      math::Row {-0.80827, -1.45677, -0.44361, 0.52068},
      math::Row {-0.07895, -0.22368, -0.05263, 0.19737},
      math::Row {-0.52256, -0.81391, -0.30075, 0.30639},
  	}
    );

  EXPECT_EQ(B, C);

  // Another check
  math::Matrix M (std::vector<math::Row> {
      math::Row {8, -5, 9, 2},
      math::Row {7, 5, 6, 1},
      math::Row {-6, 0, 9, 6},
      math::Row {-3, 0, -9, -4},
  	}
    );
  // What should be its inverse
  math::Matrix N (std::vector<math::Row> {
      math::Row {-0.15385, -0.15385, -0.28205, -0.53846},
      math::Row {-0.07692, 0.12308, 0.02564, 0.03077},
      math::Row {0.35897, 0.35897, 0.43590, 0.92308},
      math::Row {-0.69231, -0.69231, -0.76923, -1.92308},
  	}
    );

  EXPECT_EQ(inverse(M), N);

  //Yet another check
  math::Matrix X (std::vector<math::Row> {
      math::Row {9, 3, 0, 9},
      math::Row {-5, -2, -6, -3},
      math::Row {-4, 9, 6, 4},
      math::Row {-7, 6, 6, 2},
  	}
    );

  math::Matrix Y (std::vector<math::Row> {
      math::Row {-0.04074, -0.07778, 0.14444, -0.22222},
      math::Row {-0.07778, 0.03333, 0.36667, -0.33333},
      math::Row {-0.02901, -0.14630, -0.10926, 0.12963},
      math::Row {0.17778, 0.06667, -0.26667, 0.33333},
  	}
    );

  EXPECT_EQ(inverse(X), Y);

  // We multiply Matrix A by Matix B, producing Matrix C.
  // If we multiply Matrix C by inverse of Matrix B, we get back Matrix A

  math::Matrix D (std::vector<math::Row> {
      math::Row {3, -9, 7, 3},
      math::Row {3, -8, 2, -9},
      math::Row {-4, 4, 4, 1},
      math::Row {-6, 5, -1, 1},
  	}
    );

  math::Matrix E (std::vector<math::Row> {
      math::Row {8, 2, 2, 2},
      math::Row {3, -1, 7, 0},
      math::Row {7, 0, 5, 4},
      math::Row {6, -2, 0, 5},
  	}
    );

  math::Matrix F = D * E;

  EXPECT_EQ((F * inverse(E)), D);

  // But Matrix multiplication is not commutative
  EXPECT_NE((F * inverse(D)), E);
}

TEST(MatrixTransformations, Translation) {

  auto transform = math::translation(5.0, -3.0, 2.0);
  auto p = math::Point(-3.0, 4.0, 5.0);

  EXPECT_EQ(transform * p, math::Point(2.0, 1.0, 7.0));

  // The inverse of a translation matrix should moves in reverse
  auto inv = inverse(transform);

  EXPECT_EQ(inv * p, math::Point(-8.0, 7.0, 3.0));;

  // Multiplying a vector by a translation matrix should not change the vector:
  // moving around a direction does not change the direction it points to
  auto vec = math::Vector(-3.0, 4.0, 5.0);

  EXPECT_EQ(transform * vec, vec);
}

TEST(MatrixTransformations, Scaling) {

  auto transform = math::scaling(2.0, 3.0, 4.0);
  auto p = math::Point(-4.0, 6.0, 8.0);

  EXPECT_EQ(transform * p, math::Point(-8.0, 18.0, 32.0));

  // Unlike translation, scaling applies to vectors as well, changing their magnitude
  auto vec = math::Vector(-4.0, 6.0, 8.0);

  EXPECT_EQ(transform * vec, math::Vector(-8.0, 18.0, 32.0));

  // Multiplying a tuple by the inverse og a scaling scale in the opposite way (shring insted of growth and vice versa)
  auto inv = inverse(transform);

  EXPECT_EQ(inv * vec, math::Vector(-2.0, 2.0, 2.0));

  // Reflection is a transformation that takes a a points and reflects it, moves it, to the other side of an axis
  // It is essentially scaling by a negative value
  auto reflection = math::scaling(-1.0, 1.0, 1.0);
  auto p2 = math::Point(2.0, 3.0, 4.0);

  EXPECT_EQ(reflection * p2, math::Point(-2.0, 3.0, 4.0)); // p2.x is passed from positive to negative side of the x axis
}

TEST(MatrixTransformations, RotationX) {

  auto p = math::Point(0.0, 1.0, 0.0);
  auto half_quarter = math::rotation_x(M_PI / 4.0);
  auto full_quarter = math::rotation_x(M_PI / 2.0);

  EXPECT_EQ(half_quarter * p, math::Point(0.0, sqrt(2.0) / 2.0, sqrt(2.0) / 2.0));
  EXPECT_EQ(full_quarter * p, math::Point(0.0, 0.0, 1.0));

  // The inverse rotates in the opposite direction
  EXPECT_EQ(inverse(half_quarter) * p, math::Point(0.0, sqrt(2.0)/ 2.0, - sqrt(2.0) / 2.0));
}

TEST(MatrixTransformations, RotationY) {

  auto p = math::Point(0.0, 0.0, 1.0);
  auto half_quarter = math::rotation_y(M_PI / 4.0);
  auto full_quarter = math::rotation_y(M_PI / 2.0);

  EXPECT_EQ(half_quarter * p, math::Point(sqrt(2.0) / 2.0, 0.0, sqrt(2.0) / 2.0));
  EXPECT_EQ(full_quarter * p, math::Point(1.0, 0.0, 0.0));
}

TEST(MatrixTransformations, RotationZ) {

  auto p = math::Point(0.0, 1.0, 0.0);
  auto half_quarter = math::rotation_z(M_PI / 4.0);
  auto full_quarter = math::rotation_z(M_PI / 2.0);

  EXPECT_EQ(half_quarter * p, math::Point(-sqrt(2.0) / 2.0, sqrt(2.0) / 2.0, 0.0));
  EXPECT_EQ(full_quarter * p, math::Point(-1.0, 0.0, 0.0));
}

TEST(MatrixTransformations, Shearing) {

  // Moves x in proportion to y
  auto transform = math::shearing(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  auto p = math::Point(2.0, 3.0, 4.0);

  EXPECT_EQ(transform * p, math::Point(5.0, 3.0, 4.0));

  // Moves x in proportion to z
  transform = math::shearing(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(transform * p, math::Point(6.0, 3.0, 4.0));

  // Moves y in proportion to x
  transform = math::shearing(0.0, 0.0, 1.0, 0.0, 0.0, 0.0);
  EXPECT_EQ(transform * p, math::Point(2.0, 5.0, 4.0));

  // Moves y in proportion to z
  transform = math::shearing(0.0, 0.0, 0.0, 1.0, 0.0, 0.0);
  EXPECT_EQ(transform * p, math::Point(2.0, 7.0, 4.0));

  // Moves z in proportion to x
  transform = math::shearing(0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  EXPECT_EQ(transform * p, math::Point(2.0, 3.0, 6.0));
  
  // Moves z in proportion to y
  transform = math::shearing(0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
  EXPECT_EQ(transform * p, math::Point(2.0, 3.0, 7.0));
}

TEST(MatrixTransformations, ChainedTransforms) {

  // Individual transformations are applied in sequence
  auto p = math::Point(1.0, 0.0, 1.0);
  auto A = math::rotation_x(M_PI / 2.0);
  auto B = math::scaling(5.0, 5.0, 5.0);
  auto C = math::translation(10.0, 5.0, 7.0);
  // apply rotation
  auto p2 = A * p;
  EXPECT_EQ(p2, math::Point(1.0, -1.0, 0.0));
  // then scaling
  auto p3 = B * p2;
  EXPECT_EQ(p3, math::Point(5.0, -5.0, 0.0));
  // and translation
  auto p4 = C * p3;
  EXPECT_EQ(p4, math::Point(15.0, 0.0, 7.0));

  // Can also chain them:
  // Warning: operations must be chained in reverse order, matrix multiplication is not a commutative operation
  auto T = C * B * A;
  EXPECT_EQ(T * p, math::Point(15.0, 0.0, 7.0));
}

TEST(MatrixTest, ViewTransform) {

  // The transformation matrix for the default orientation
  // view parameters don't require anything to be scaled, rotated or translated --> it is the identiy matrix
  auto from =  math::Point(0, 0, 0);
  auto to = math::Point(0, 0, -1);
  auto up = math::Vector(0, 1, 0);

  auto view_t = math::view_transform(from, to, up);
  EXPECT_EQ(view_t, math::IDENTITY_MATRIX);

  // A view transformation matrix looking in positive z direction
  // In this case, it is as if we're looking into a mirror: x and z axes are inversed
  from =  math::Point(0, 0, 0);
  to = math::Point(0, 0, 1);
  up = math::Vector(0, 1, 0);

  view_t = math::view_transform(from, to, up);
  EXPECT_EQ(view_t, math::scaling(-1, 1, -1));

  // The view transform matrix moves the world, not the eye
  from =  math::Point(0, 0, 8);
  to = math::Point(0, 0, 0);
  up = math::Vector(0, 1, 0);

  view_t = math::view_transform(from, to, up);
  EXPECT_EQ(view_t, math::translation(0, 0, -8)); // thw world is pushed away from the eye

  // An arbitrary view transformation
  from = math::Point(1, 3, 2);
  to = math::Point(4, -2, 8);
  up = math::Vector(1, 1, 0);

  view_t = math::view_transform(from, to, up);
  EXPECT_EQ(view_t, math::Matrix(std::vector<math::Row> {
	math::Row {-0.50709, 0.50709, 0.67612, -2.36643},
	math::Row {0.76772, 0.60609, 0.12122, -2.82843},
	math::Row {-0.35857, 0.59761, -0.71714, 0.0},
	math::Row {0.0, 0.0, 0.0, 1.0}
      })
    );
}

