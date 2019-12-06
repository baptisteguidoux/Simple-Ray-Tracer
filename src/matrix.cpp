#include <cmath>
#include <vector>
#include <stdexcept>
#include <ostream>

#include "matrix.hpp"
#include "tuple.hpp"


namespace math {
  
  Matrix::Matrix(const std::vector<std::vector<double>>& c) : content{c} {

    // Should check the size of each row to see if the same?
    height = content.size();
    width = content.at(0).size();
  }
  
  Matrix::Matrix(const Matrix& orig) :
    height {orig.height}, width {orig.width}, content {orig.content} {};

  Matrix::Matrix(const size_t h, const size_t w) : height {h}, width {w} {

    content = std::vector<std::vector<double>>(height, std::vector<double>(width));
  }

  const double& Matrix::operator()(const size_t row, const size_t col) const {

    return content.at(row).at(col);
  }

  double& Matrix::operator()(const size_t row, const size_t col) {

    return content.at(row).at(col);
  }

  bool operator==(const Matrix& a, const Matrix& b) {

    if (a.width != b.width || a.height != b.height)
      return false;

    for (size_t row = 0; row < a.height; row++)
      for (size_t col = 0; col < a.width; col++)
	if (! math::almost_equal(a(row, col), b(row, col)))
	  return false;

    return true;
  }

  bool operator!=(const Matrix& a, const Matrix& b) {

    return ! (a == b);
  }

  Matrix operator*(const Matrix& a, const Matrix& b) {

    if (a.width != b.height)
      throw std::runtime_error {"Matrix A width is not equal to Matrix B height. Cannot multiply"};

    Matrix result(a.height, b.width);

    for (size_t row = 0; row < result.height; row++) {
      for (size_t col = 0; col < result.width; col++) {
	double pos_sum = 0.0;
	for (size_t i = 0; i < a.width; i++) // a.width or b.height, since they're the same size
	  pos_sum += a(row, i) * b(i, col);
	result(row, col) = pos_sum;
      }
    }

    return result;
  }

  math::Tuple operator*(const Matrix& matrix, const math::Tuple& tuple) {

    // Tuple size is 4 (for the moment at least)
    if (matrix.height != 4)
      throw std::runtime_error {"The Matrix must have a height of 4 when multiplying a Matrix and a tuple "}; // We'll see if need to mult with other sizes

    math::Tuple result;

    for (size_t row = 0; row < matrix.height; row++) {

      result[row] = (matrix(row, 0) * tuple.x +
		     matrix(row, 1) * tuple.y +
		     matrix(row, 2) * tuple.z +
		     matrix(row, 3) * tuple.w);
    }

    return result;
  }

  Matrix operator/(const Matrix& source, const double divisor) {

    Matrix result = source;

    for (size_t row = 0; row < result.height; row++)
      for (size_t col = 0; col < result.width; col++)
	result(row, col) = result(row, col) / divisor;
    
    return result;
  }

  Matrix transpose(const Matrix& matrix) {

    Matrix result(matrix.width, matrix.height); // Inverse width and height

    for (size_t row = 0; row < result.height; row++)
      for (size_t col = 0; col < result.width; col++)
  	result(row, col) = matrix(col, row); // simply inverse

    return result;
  }  
      
  double cofactor(const Matrix& matrix, const size_t row_pos, const size_t col_pos) {

    // Cofactors are minors who may have their sign changed
    // If row + col is odd --> negate the minor
  
    if ((row_pos + col_pos) % 2 == 0)
      return minor(matrix, row_pos, col_pos);
    else
      return - minor(matrix, row_pos, col_pos);
  }  

  double determinant(const Matrix& matrix) {


    if (matrix.height == 2 || matrix.width == 2)
      return matrix(0, 0) * matrix(1, 1) - matrix(0, 1) * matrix(1, 0);

    // When the matrix is > 2x2, it finding the derminant works recursively: determinant() -> cofactor() -> minor() -> determinant()
    // First look at any row or column. Let's choose the first row
    // No matter which row!
    static constexpr size_t row = 0;
    double result = 0;
    for (size_t col = 0; col < matrix.content.at(row).size(); col++)
      // for each of this elem, multiply it by its cofactor
      result += matrix(row, col) * cofactor(matrix, row, col);
 
    return result;
  }

  Matrix submatrix(const Matrix& matrix, const size_t row_pos, const size_t col_pos) {

    Matrix result (matrix); // Copy

    result.content.erase(result.content.begin() + row_pos);
    for (auto& row : result.content)
      row.erase(row.begin() + col_pos);

    // !! New heights and width
    result.height--;
    result.width--;
  
    return result;
  }

  double minor(const Matrix& matrix, const size_t row_pos, const size_t col_pos) {

    // The minor of an element at row i, col j is determinant of the submatrix at i,j

    return determinant(submatrix(matrix, row_pos, col_pos));
  }

  bool is_invertible(const Matrix& matrix) {

    return (determinant(matrix) != 0.0);
  }

  Matrix inverse(const Matrix& source) {

    /*To inverse a matrix, each element is converted to its cofactor (matrix of cofactors),
      then the result is transposed, and finally each member is divided by the determinant of the original matrix */
    
    if (!is_invertible(source))
      throw std::runtime_error {"Cannot inverse matrix"};
    
    Matrix result = source;
    const double source_det = determinant(source);
    for (size_t row = 0; row < source.height; row++)
      for (size_t col = 0; col < source.width; col++)
  	// Notice that we inverse row, col in result to directly transpose the result
  	result(col, row) = cofactor(source, row, col) / source_det;

    return result;
  }
  
  std::ostream& operator<<(std::ostream& os, const Matrix& matrix) {

    for (const auto& line : matrix.content) {
      for (const auto& el : line)
  	os << "| " << el << " ";
      os << "|\n";
    }
  
    return os;
  }

  Matrix translation(const double x, const double y, const double z) {

    auto result = IDENTITY_MATRIX;

    /* Translation matrix can be constructed simply from the identity matrix:
       1 0 0 x
       0 1 0 y
       0 0 1 z
       0 0 0 1

       When multiplied by a vector, the 0 w component of it causes those translation values to disappear!
       With a point with 1 at w, the tranlation is applied to the point
     */
    result(0, 3) = x;
    result(1, 3) = y;
    result(2, 3) = z;

    return result;
  }

  Matrix scaling(const double x, const double y, const double z) {

    auto result = IDENTITY_MATRIX;
    
    /* Scaling matrix can be constructed simply from the identity matrix:
       x 0 0 0
       0 y 0 0
       0 0 z 0
       0 0 0 1
     */
    
    result(0, 0) = x;
    result(1, 1) = y;
    result(2, 2) = z;

    return result;
  }

  Matrix rotation_x(const double radians) {
    // sine and cosine of std::math take an argument given in radians

    auto result = IDENTITY_MATRIX;

    /* Rotation X:
       1 0      0       0
       0 cos(r) -sin(r) 0
       0 sin(r) cos(r)  0
       0 0      0       1
     */
    
    result(1, 1) = std::cos(radians);
    result(1, 2) = - std::sin(radians);
    result(2, 1) = std::sin(radians);
    result(2, 2) = std::cos(radians);
    
    return result;
  }

  Matrix rotation_y(const double radians) {

    auto result = IDENTITY_MATRIX;

    /* Rotation Y:
       cos(r)  0   sin(r) 0
       0       1   0      0
       -sin(r) 0   cos(r) 0
       0       0   0      1
     */
    
    result(0, 0) = std::cos(radians);
    result(0, 2) = std::sin(radians);
    result(2, 0) = - std::sin(radians);
    result(2, 2) = std::cos(radians);

    return result;
  }

  Matrix rotation_z(const double radians) {

    auto result = IDENTITY_MATRIX;

    /* Rotation Z:
       cos(r)  -sin(r) 0   0
       sin(r)  cos(r)  0   0
       0       0       1   0
       0       0       0   1
     */

    result(0, 0) = std::cos(radians);
    result(0, 1) = - std::sin(radians);
    result(1, 0) = std::sin(radians);
    result(1, 1) = std::cos(radians);

    return result;
  }

  Matrix shearing(const double x_y, const double x_z, const double y_x,
		  const double y_z, const double z_x, const double z_y) {

    // a component is affected by either of the other two components, for instance: x is moved in proportion of y
    // it represents the amount by which to multiply y before adding it to x

    auto result = IDENTITY_MATRIX;

    /* Shearing matrix:
       1   x_y x_z 0
       y_x 1   y_z 0
       z_x z_y 1   0
       0   0   0   1
     */
    
    result(0, 1) = x_y;
    result(0, 2) = x_z;
    result(1, 0) = y_x;
    result(1, 2) = y_z;
    result(2, 0) = z_x;
    result(2, 1) = z_y;

    return result;
  }

  Matrix view_transform(const Tuple& from, const Tuple& to, const Tuple& up_vec) {

    auto forward_vec = normalize(to - from);

    auto left_vec = cross(forward_vec, normalize(up_vec));

    // This allows the original up_vec to be only approximatively perpendicular to the viewing direction
    auto true_up_vec = cross(left_vec, forward_vec);

    auto orientation = Matrix(std::vector<Row> {
  	Row {left_vec.x, left_vec.y, left_vec.z, 0},
  	Row {true_up_vec.x, true_up_vec.y, true_up_vec.z, 0},
  	Row {-forward_vec.x, -forward_vec.y, -forward_vec.z, 0},
  	Row {0, 0, 0, 1}
      });

    // Move the scene into place before orienting it
    return orientation * translation(-from.x, -from.y, -from.z);
  }
  
}

