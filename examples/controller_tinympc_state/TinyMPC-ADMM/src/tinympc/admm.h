#ifndef ADMM_H
# define ADMM_H

#include "types.h"
#include "utils.h"
#include "lqr.h"
#include "constraint_linear.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

enum tiny_ErrorCode tiny_SolveAdmm(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode UpdatePrimal(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode UpdateSlackDual(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode ComputePrimalResidual(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode ComputeDualResidual(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode ComputeObjectiveValue(tiny_AdmmWorkspace* work);

int CheckTermination(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode UpdateInfo(tiny_AdmmWorkspace* work,
                               int                 iter,
                               int                 compute_objective);

enum tiny_ErrorCode tiny_WarmStartInput(tiny_AdmmWorkspace* work, Eigen::VectorMf* U);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef ADMM_H