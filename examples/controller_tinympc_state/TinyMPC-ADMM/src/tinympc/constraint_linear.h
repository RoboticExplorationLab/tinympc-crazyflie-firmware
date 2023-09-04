#ifndef CONSTRAINT_LINEAR_H
# define CONSTRAINT_LINEAR_H

#include "types.h"
#include "utils.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

enum tiny_ErrorCode tiny_SetInputBound(tiny_AdmmWorkspace* work, Eigen::MatrixMf* Acu, Eigen::VectorMf* lcu, Eigen::VectorMf* ucu);

// enum tiny_ErrorCode tiny_SetStateBound(tiny_AdmmWorkspace* work, Eigen::MatrixNf* Acx, Eigen::VectorNf* lcx, Eigen::VectorNf* ucx);

enum tiny_ErrorCode tiny_SetStateConstraint(tiny_AdmmWorkspace* work, Eigen::VectorNf* Acx, Eigen::VectorNf* lcx, Eigen::VectorNf* ucx);
// enum tiny_ErrorCode tiny_ProjectInput(tiny_AdmmWorkspace* work);

int IsConstrained(tiny_AdmmWorkspace* work);


# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef CONSTRAINT_LINEAR_H