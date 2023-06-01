#ifndef UTILS_H
# define UTILS_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include "types.h"
#include "constants.h"
#include "slap/slap.h"

// void PrintLine(void);

// /**
//  * Print header with data to be displayed per iteration
//  */
// void PrintHeader(void);

// /**
//  * Print iteration summary
//  * @param work current workspace
//  */
// void PrintIteration(tiny_AdmmWorkspace *work);

// /**
//  * Print summary when algorithm terminates
//  * @param info   info structure
//  */
// void PrintSummary(tiny_AdmmInfo *info);

// //========================================
// // Print matrix with its name (dummy)
// //========================================
// #define PrintMatrix(mat)      \
//   {                          \
//     printf("%s = \n", #mat); \
//     slap_PrintMatrix(mat);   \
//   }

// #define PrintMatrixT(mat)                   \
//   {                                        \
//     printf("%s = \n", #mat);               \
//     slap_PrintMatrix(slap_Transpose(mat)); \
//   }

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
// #define PrintMatrixInfo(mat)      \
//   {                          \
//     printf("%s info: \n", #mat); \
//     printf(" Dims: (%d, %d)\n", mat.rows, mat.cols);   \
//     printf(" Data: "); \
//     for (int imat = 0; imat < mat.cols * mat.rows; ++imat) { \
//       printf("%.4f, ", mat.data[imat]); \
//     } \
//     printf("\n");\
//   }

// //========================================
// // Print model info
// //========================================
// #define PrintModelInfo(model)      \
//   { \
//     printf("Model info: \n"); \
//     printf(" States: %d, inputs: %d, dt: %f\n", model.nstates, model.ninputs, model.dt);   \
//     printf(" LTV: %d, affine: %d\n", model.ltv, model.affine); \
//   }

// void PrintSolveInfo(tiny_AdmmWorkspace* work);

//========================================
// Clamp the inputs to within min max value,
// will modify the provided array
//========================================
void tiny_Clamps(sfloat* arr, const sfloat* min, const sfloat* max,
                 const int N);

void tiny_Clamp(sfloat* arr, const sfloat min, const sfloat max, const int N);

void tiny_ClampMatrix(Matrix* mat, const Matrix min, const Matrix max);

void tiny_ShiftFill(Matrix* mats, const int length);

void tiny_ShiftFillWith(Matrix* mats, const sfloat* x, const int length);


//========================================
// Raw matrix operators, ignore all metadata
//========================================
void SwapVectors(sfloat **a, sfloat **b);
void MatAdd(Matrix C, Matrix A, Matrix B, sfloat alpha);
void MatCpy(Matrix des, Matrix src);
void MatScale(Matrix A, sfloat alpha);
void MatMulAdd(Matrix C, Matrix A, Matrix B, sfloat alpha, sfloat beta);
void MatMulAdd2(Matrix D, Matrix C, Matrix A, Matrix B, sfloat alpha, sfloat beta);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef UTILS_H