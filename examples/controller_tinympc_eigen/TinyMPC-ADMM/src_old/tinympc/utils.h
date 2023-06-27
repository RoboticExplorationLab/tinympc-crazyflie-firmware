#ifndef UTILS_H
# define UTILS_H

#include <iostream>
#include <Eigen.h>

#include <stdlib.h>
#include <string.h>

#include "types.h"
#include "constants.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

void PrintLine(void);

/**
 * Print header with data to be displayed per iteration
 */
void PrintHeader(void);

/**
 * Print iteration summary
 * @param work current workspace
 */
void PrintIteration(tiny_AdmmWorkspace *work);

/**
 * Print summary when algorithm terminates
 * @param info   info structure
 */
void PrintSummary(tiny_AdmmInfo *info);

//========================================
// Print matrix with its name (dummy)
//========================================
#define PrintMatrix(mat)      \
  {                          \
    printf("%s = \n", #mat); \
    std::cout << mat << std::endl;   \
  }

#define PrintMatrixT(mat)                   \
  {                                        \
    printf("%s = \n", #mat);               \
    std::cout << (mat).transpose() << std::endl;  \
  }

//========================================
// Return length of an array
//========================================
#define T_ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))

//========================================
// Initialize memory with zeros
//========================================
#define T_INIT_ZEROS(data) (memset(data, 0, sizeof(data)))  

//========================================
// Return a random noise from percentage
//========================================
#define T_NOISE(percent) (((2 * ((float)rand() / RAND_MAX)) - 1) / 100 * percent)

//========================================
// Maximum
//========================================
# ifndef T_MAX
#  define T_MAX(a, b) (((a) > (b)) ? (a) : (b))
# endif /* ifndef T_MAX */

//========================================
// Minimum
//========================================
# ifndef T_MIN
#  define T_MIN(a, b) (((a) < (b)) ? (a) : (b))
# endif /* ifndef T_MIN */

//========================================
// Absolute
//========================================
# ifndef T_ABS
#  define T_ABS(x) (((x) < 0) ? -(x) : (x))
# endif /* ifndef T_ABS */

//========================================
// Print matrix info
//========================================
#define PrintMatrixInfo(mat)      \
  {                          \
    printf("%s info: \n", #mat); \
    printf(" Dims: (%d, %d)\n", mat.rows(), mat.cols());   \
    printf(" Data: "); \
    std::cout << mat << std::endl; \
  }

//========================================
// Print model info
//========================================
#define PrintModelInfo(model)      \
  { \
    printf("Model info: \n"); \
    printf(" States: %d, inputs: %d, dt: %f\n", model.nstates, model.ninputs, model.dt);   \
    printf(" LTV: %d, affine: %d\n", model.ltv, model.affine); \
  }

void PrintSolveInfo(tiny_AdmmWorkspace* work);

// //========================================
// // Read data from file
// //========================================
// int tiny_ReadData(const char* filename, float* des, const int size,
//                   bool verbose);

// //========================================
// // Read data from file and copy the last knot point into
// // remaining space of the array. Useful for extend horizon at the end.
// //========================================
// int tiny_ReadData_Extend(const char* filename, float* des, const int stride,
//                          const int size, bool verbose);

// //========================================
// // Read data from file and copy the goal state into
// // remaining space of the array. Useful for extend horizon at the end.
// //========================================
// int tiny_ReadData_ExtendGoal(const char* filename, float* des,
//                              const float* xf, const int stride, const int size,
//                              bool verbose);

// //========================================
// // Clamp the inputs to within min max value,
// // will modify the provided array
// //========================================
// void tiny_Clamps(float* arr, const float* min, const float* max,
//                  const int N);

// void tiny_Clamp(float* arr, const float min, const float max, const int N);

// void tiny_ClampMatrix(Matrix* mat, const Matrix min, const Matrix max);

// void tiny_ShiftFill(Matrix* mats, const int length);

// void tiny_ShiftFillWith(Matrix* mats, const float* x, const int length);


//========================================
// Raw matrix operators, ignore all metadata
//========================================
void SwapVectors(float **a, float **b);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef UTILS_H