/**
 * @file function_mapping.h
 * @author Brian Jackson (bjack205@gmail.com)
 * @brief Method for applying functions element-wise
 * @date 2022-01-30
 *
 * @copyright Copyright (c) 2022
 *
 * @addtogroup Advanced
 * @{
 */

#pragma once

#include "matrix.h"

/**
 * @brief Applies a function element-wise to every element in the matrix
 *
 * Applies `mat[k] = function(mat[k])` for all elements
 *
 * @param mat A valid matrix (can be dense or strided)
 * @param function A function pointer that takes and returns a sfloat
 * @return
 */
enum slap_ErrorCode slap_Map(Matrix mat, sfloat (*function)(sfloat));

/**
 * @brief Applies a function element-wise to a pair of matrices
 *
 * Given a function pointer \f$c = f(x,y)\f$, this function applies
 * this operation element-wise, storing the output in C.
 *
 * The matrices must all be the same size (but can have different strides).
 *
 * The matrices can be aliased.
 *
 * # Example
 * ```sfloat c
 * sfloat binary_op(sfloat x, sfloat y) {
 *   return 2 * x - y * x;
 * }
 *
 * int main() {
 * // Initialize matrices ...
 * slap_BinaryMap(C, A, B, binary_op);
 * // Clean-up operations ...
 * }
 * ```
 * For more details, see the `BinaryMap` test in `test/matrix_test.cpp`.
 *
 * @param[out] C Destination matrix
 * @param[in] A Input matrix, provides first arguments to function
 * @param[in] B Input matrix, provides second arguments to function
 * @param function A function that takes two sfloat and returns a sfloat
 */
enum slap_ErrorCode slap_BinaryMap(Matrix C, Matrix A, Matrix B,
                                   sfloat (*function)(sfloat, sfloat));
