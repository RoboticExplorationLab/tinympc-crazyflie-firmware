#include "utils.h"

#ifdef __cplusplus
extern "C" {
#endif

void PrintSolveInfo(tiny_AdmmWorkspace* work) {
  tiny_AdmmInfo* info = work->info;
  printf("Solve info: \n");
  printf(" Status: %d\n", info->status_val);
  printf(" Iter: %d\n", info->iter);   
  printf(" Obj value: %f\n", info->obj_val);
  printf(" Primal res: %f, dual res: %f\n", info->pri_res, info->dua_res);
}

void PrintLine(void) {
  char  the_line[HEADER_LINE_LEN + 1];
  int i;

  for (i = 0; i < HEADER_LINE_LEN; ++i) the_line[i] = '-';
  the_line[HEADER_LINE_LEN] = '\0';
  printf("%s\n", the_line);
}

void PrintHeader(void) {
  // Different indentation required for windows
  printf("iter   ");
  // Main information
  printf("objective    pri res    dua res    rho");
  printf("\n");
}

void PrintIteration(tiny_AdmmWorkspace *work) {
  tiny_AdmmInfo *info;

  info = work->info;

  printf("%4i",     (int)info->iter);
  printf(" %12.4e", info->obj_val);
  printf("  %9.2e", info->pri_res);
  printf("  %9.2e", info->dua_res);
  printf("  %9.2e", work->rho);
  printf("\n");
}

void PrintSummary(tiny_AdmmInfo *info) {
  printf("\n"); // Add space after iterations

  printf("status:               %i\n", info->status_val);

  printf("number of iterations: %i\n", (int)info->iter);

  if (info->status_val == TINY_SOLVED) {
    printf("optimal objective:    %.4f\n", info->obj_val);
  }
  printf("\n");
}

// //========================================
// // Read data from file
// //========================================
// int tiny_ReadData(const char* filename, float* des, const int size,
//                   bool verbose) {
//   FILE* input;
//   int i;

//   input = fopen(filename, "r");
//   if (!input) {
//     if (verbose == true)
//       fprintf(stderr, "Cannot open %s: %s.\n", filename, strerror(errno));
//     return EXIT_FAILURE;
//   }

//   for (i = 0; i < size; ++i) {
//     if (fscanf(input, "%lf ", &(des[i])) != 1) {
//       if (verbose == true) fprintf(stderr, "Invalid data in %s.\n", filename);
//       fclose(input);
//       return EXIT_FAILURE;
//     }

//     if (verbose == true) printf("Read %lf from %s.\n", des[i], filename);
//   }

//   if (ferror(input)) {
//     fclose(input);
//     if (verbose == true) fprintf(stderr, "Error reading %s.\n", filename);
//     return EXIT_FAILURE;
//   }
//   if (fclose(input)) {
//     if (verbose == true) fprintf(stderr, "Error closing %s.\n", filename);
//     return EXIT_FAILURE;
//   }

//   if (verbose == true) printf("All floats read successfully.\n");

//   return EXIT_SUCCESS;
// }

// //========================================
// // Read data from file and copy the last knot point into
// // remaining space of the array. Useful for extend horizon at the end.
// //========================================
// int tiny_ReadData_Extend(const char* filename, float* des, const int stride,
//                          const int size, bool verbose) {
//   FILE* input;
//   int i;
//   int k = 0;
//   input = fopen(filename, "r");
//   if (!input) {
//     if (verbose == true)
//       fprintf(stderr, "Cannot open %s: %s.\n", filename, strerror(errno));
//     return EXIT_FAILURE;
//   }

//   for (i = 0; i < size; ++i) {
//     if (fscanf(input, "%lf ", &(des[i])) != 1) {
//       if (verbose == true) fprintf(stderr, "Invalid data in %s.\n", filename);
//       fclose(input);
//       break;
//     }

//     if (verbose == true) printf("Read %lf from %s.\n", des[i], filename);

//     k += 1;
//   }

//   if (verbose == true)
//     printf("All floats read successfully and now extend.\n");

//   int remain_cnt = (size - k) / stride;  // # of remaining chunks
//   for (i = 0; i < remain_cnt; i += 1) {
//     for (int j = 0; j < stride; j += 1) {
//       des[k + j + i * stride] = des[k + j - stride];  // copy
//     }
//   }

//   return EXIT_SUCCESS;
// }

// //========================================
// // Read data from file and copy the goal state into
// // remaining space of the array. Useful for extend horizon at the end.
// //========================================
// int tiny_ReadData_ExtendGoal(const char* filename, float* des,
//                              const float* xf, const int stride, const int size,
//                              bool verbose) {
//   FILE* input;
//   int i;
//   int k = 0;
//   input = fopen(filename, "r");
//   if (!input) {
//     if (verbose == true)
//       fprintf(stderr, "Cannot open %s: %s.\n", filename, strerror(errno));
//     return EXIT_FAILURE;
//   }

//   for (i = 0; i < size; ++i) {
//     if (fscanf(input, "%lf ", &(des[i])) != 1) {
//       if (verbose == true) fprintf(stderr, "Invalid data in %s.\n", filename);
//       fclose(input);
//       break;
//     }

//     if (verbose == true) printf("Read %lf from %s.\n", des[i], filename);

//     k += 1;
//   }

//   if (verbose == true)
//     printf("All floats read successfully and now extend.\n");

//   int remain_cnt = (size - k) / stride;  // # of remaining chunks
//   for (i = 0; i < remain_cnt; i += 1) {
//     for (int j = 0; j < stride; j += 1) {
//       des[k + j + i * stride] = xf[j];  // copy
//     }
//   }

//   return EXIT_SUCCESS;
// }

//========================================
// Clamp the inputs to within min max value
//========================================
// void tiny_Clamps(float* arr, const float* min, const float* max,
//                  const int N) {
//   for (int k = 0; k < N; ++k) {
//     arr[k] = (arr[k] > max[k]) ? max[k] : ((arr[k] < min[k]) ? min[k] : arr[k]);
//   }
// }

// void tiny_Clamp(float* arr, const float min, const float max, const int N) {
//   for (int k = 0; k < N; ++k) {
//     arr[k] = (arr[k] > max) ? max : ((arr[k] < min) ? min : arr[k]);
//   }
// }

// // Clamp all data for matrix or vector
// void tiny_ClampMatrix(Matrix* mat, const Matrix min, const Matrix max) {
//   tiny_Clamps(mat->data, min.data, max.data, (mat->rows) * (mat->cols));
// }

// void tiny_ShiftFill(Matrix* mats, const int length) {
//   for (int k = 0; k < length - 1; ++k) {
//     slap_Copy(mats[k], mats[k + 1]);
//   }
//   slap_Copy(mats[length - 1], mats[length - 2]);
// }

// void tiny_ShiftFillWith(Matrix* mats, const float* x, const int length) {
//   for (int k = 0; k < length - 1; ++k) {
//     slap_Copy(mats[k], mats[k + 1]);
//   }
//   slap_Copy(mats[length - 1], x);
// }

#ifdef __cplusplus
}
#endif