/*! \file matrix.hpp
 */

#ifndef MATRIX_H
#define MATRIX_H


#undef minor // GNU defines a minor macro


#include <vector>

#include "tuple.hpp"


/*! \namespace math
 */
namespace math {

  /*! \struct Matrix
   */
  struct Matrix {

    size_t height;
    size_t width;
    std::vector<std::vector<double>> content;

    /* Matrix constructor
     * \param c 2d vector of tuple
     */
    Matrix(const std::vector<std::vector<double>>& c);

    /* Matrix constructor
     * \param orig source matrix
     */    
    Matrix(const Matrix& orig);

    /* Matrix constructor
     *  \brief Build a matrix full of 0
     *  \param h matrix's height
     *  \param w matrix's width     
     */        
    Matrix(const size_t h, const size_t w);

    /*! \fn const double& operator() (const size_t row, const size_t col) const
     *  \brief Get the value located at the given position
     *  \param row row index
     *  \param col column index
     *  \return the element at the given location
     */ 
    const double& operator()(const size_t row, const size_t col) const;

    /*! \fn double& Matrix::operator()(const size_t row, const size_t col)
     *  \brief Get the value located at the given position
     *  \param row row index
     *  \param col column index
     *  \return the element at the given location
     */
    double& operator()(const size_t row, const size_t col);
    
  };

  /*! \fn bool operator==(const Matrix& a, const Matrix& b)
   *  \brief Compare the equality of two Matrix
   *  \param a a Matrix
   *  \param b another Matrix
   *  \return true if the two Matrix are equal
   */
    bool operator==(const Matrix& a, const Matrix& b);

  /*! \fn bool operator!=(const Matrix& a, const Matrix& b)
   *  \brief Compare the unequality of two Matrix
   *  \param a a Matrix
   *  \param b another Matrix
   *  \return true if the two Matrix are not equal
   */
  bool operator!=(const Matrix& a, const Matrix& b);

  /*! \fn Matrix operator*(const Matrix& a, const Matrix& b)
   *  \brief Multiply two matrices together
   *  \param a a Matrix
   *  \param b another Matrix
   *  \return The product of the two matrices
   */  
  Matrix operator*(const Matrix& a, const Matrix& b);

  /*! \fn Matrix operator*(const Matrix& a, const Matrix& b)
   *  \breif Multiply a Matrix with a Tuple
   *  \param matrix a Matrix
   *  \param tuple a Tuple
   *  \return A new Tuple <- Matrix * Tuple
   */    
  math::Tuple operator*(const Matrix& matrix, const math::Tuple& tuple);

  /*! \fn Matrix operator/(const Matrix& source, const double divisor)
   *  \brief Divides a Matrix by a scalar
   *  \param source a Matrix
   *  \param divisor a scalar value
   *  \return A new Matrix
   */   
  Matrix operator/(const Matrix& source, const double divisor);

  /*! \fn Matrix transpose(const Matrix& matrix)
   *  \brief Transposes a Matrix, ie rows become columns
   *  \param matrix a Matrix
   *  \return A new Matrix
   */     
  Matrix transpose(const Matrix& matrix);

  /*! \fn Matrix submatrix(const Matrix& matrix, const size_t row_pos, const size_t col_pos)
   *  \brief Removes the row at pos row_pos, the column at col_pos from the input matrix
   *  \param matrix a Matrix
   *  \param row_pos row to del
   *  \param col_pos col to del
   *  \return A new, smaller, Matrix
   */  
  Matrix submatrix(const Matrix& matrix, const size_t row_pos, const size_t col_pos);

  /*! \fn double determinant(const Matrix& matrix)
   *  \brief Computes the given Matrix's determinant, useful to inverse a Matrix
   *  \param matrix a Matrix 
   *  \return the Matrix's determinant
   */
  double determinant(const Matrix& matrix);
  
  /*! \fn double minor(const Matrix& matrix, const size_t row_pos, const size_t col_pos)
   *  \brief The minor of an element at row i, col j is determinant of the submatrix at i,j
   *  Useful to compute Matrix's inverse
   *  \param row_pos element row position
   *  \param col_pos element column position
   *  \return the minor (a scalar value)
   */									
  double minor(const Matrix& matrix, const size_t row_pos, const size_t col_pos);

  /*! \fn double cofactor(const Matrix& matrix, const size_t row_pos, const size_t col_pos)
   *  \brief Cofactors are minors who may have their sign changed (if row + col is odd --> negate the minor)
   *  Useful to compute Matrix's inverse
   *  \param row_pos element row position
   *  \param col_pos element column position
   *  \return the cofactor (a scalar value)
   */			  
  double cofactor(const Matrix& matrix, const size_t row_pos, const size_t col_pos);

  /*! \fn bool is_invertible(const Matrix& matrix)
   *  \brief Check if the Matrix can be inverted
   *  \param matrix a Matrix
   *  \return true if the Matrix can be inverted
   */	  
  bool is_invertible(const Matrix& matrix);

  /*! \fn Matrix inverse(const Matrix& source)
   *  \brief Inverse the Matrix (better use is_invertible(const Matrix& matrix) before!)
   *  \param matrix a Matrix
   *  \return A new Matrix, the input inverted
   */  
  Matrix inverse(const Matrix& matrix);

   /*! \fn std::ostream& operator<<(std::ostream& os, const Matrix& matrix)
    *  \brief Displays a Matrix
    *  \param os Output stream
    *  \param matrix a Matrix
    *  \return output stream
    */  
  std::ostream& operator<<(std::ostream& os, const Matrix& matrix);

  /*! \fn Matrix translation(const double x, const double y, const double z)
   *  \brief Builds a translation Matrix
   *  \param x translation on the x axis
   *  \param y translation on the y axis
   *  \param z translation on the z axis
   *  \return a translation Matrix
   */
  Matrix translation(const double x, const double y, const double z);

  /*! \fn Matrix scaling(const double x, const double y, const double z)
   *  \brief Builds a scaling Matrix
   *  \param x scaling on the x axis
   *  \param y scaling on the y axis
   *  \param z scaling on the z axis
   *  \return a scaling Matrix
   */  
  Matrix scaling(const double x, const double y, const double z);

  /*! \fn Matrix rotation_x(const double radians)
   *  \brief Builds a rotation Matrix on the x axis
   *  \param radians the rotation angle
   *  \return a rotation
   */    
  Matrix rotation_x(const double radians);

  /*! \fn Matrix rotation_y(const double radians)
   *  \brief Builds a rotation Matrix on the y axis
   *  \param radians the rotation angle
   *  \return a rotation Matrix
   */      
  Matrix rotation_y(const double radians);

  /*! \fn Matrix rotation_z(const double radians)
   *  \brief Builds a rotation Matrix on the z axis
   *  \param radians the rotation angle
   *  \return a rotation Matrix
   */      
  Matrix rotation_z(const double radians);

 /*! \fn Matrix shearing(const double x_y, const double x_z, const double y_x, const double y_z, const double z_x, const double z_y)
  *  \brief Moves an axis in proportion of another
  *  \param x_y
  *  \param x_z
  *  \param y_x
  *  \param y_z
  *  \param z_x
  *  \param z_y
  *  \return a shearing Matrix
  */  
  Matrix shearing(const double x_y, const double x_z, const double y_x,
		  const double y_z, const double z_x, const double z_y);

  /*! \fn Matrix view_transform(const Tuple& from, const Tuple& to, const Tuple& up_vec)
   *  \param from position of the eye
   *  \param to where the eye looks
   *  \param up_vec indicates which direction is up
   *  \return a view transform Matrix
   */
  Matrix view_transform(const Tuple& from, const Tuple& to, const Tuple& up_vec);  
  
}

#endif

#ifndef RAY_MATRIX_CONSTANTS
#define RAY_MATRIX_CONSTANTS

namespace math {

  typedef std::vector<double> Row;

  static const Matrix IDENTITY_MATRIX (std::vector<Row> {
       math::Row {1, 0, 0, 0},
       math::Row {0, 1, 0, 0},
       math::Row {0, 0, 1, 0},
       math::Row {0, 0, 0, 1},      
  	 }
    );
  
}

#endif

