#ifndef AUXIL_H
# define AUXIL_H

#include <Eigen.h>
#include "types.h"
#include "utils.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

enum tiny_ErrorCode tiny_InitSettings(tiny_AdmmSettings* stgs);

enum tiny_ErrorCode tiny_SetUnconstrained(tiny_AdmmSettings* stgs);

// enum tiny_ErrorCode tiny_InitData(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_InitDataCost(tiny_AdmmWorkspace* work, 
Eigen::MatrixNf* Q, Eigen::VectorNf* q, Eigen::MatrixMf* R, Eigen::VectorMf* r, Eigen::VectorMf* r_tilde);

enum tiny_ErrorCode tiny_InitSolution(tiny_AdmmWorkspace* work,
Eigen::VectorNf* X, Eigen::VectorMf* U,
Eigen::VectorNf* YX, Eigen::VectorMf* YU, Eigen::VectorNf* YG,
Eigen::MatrixMNf* Kinf, Eigen::VectorMf* d, 
Eigen::MatrixNf* Pinf, Eigen::VectorNf* p);

enum tiny_ErrorCode tiny_InitSolutionStretch(tiny_AdmmWorkspace* work,
Eigen::MatrixMNf* Kinf, Eigen::MatrixNf* Pinf);

enum tiny_ErrorCode tiny_InitWorkspace(tiny_AdmmWorkspace* work,
                                       tiny_AdmmInfo* info,
                                       tiny_Model* model,
                                       tiny_AdmmData* data,
                                       tiny_AdmmSolution* soln,
                                       tiny_AdmmSettings* stgs);

enum tiny_ErrorCode tiny_InitWorkspaceTemp(tiny_AdmmWorkspace* work, Eigen::VectorMf* Qu,
Eigen::VectorMf* ZU, Eigen::VectorMf* ZU_new, Eigen::VectorNf* ZX, Eigen::VectorNf* ZX_new);

// enum tiny_ErrorCode tiny_EvalPrimalCache(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_InitPrimalCache(tiny_AdmmWorkspace* work, 
Eigen::MatrixMf* Quu_inv_data, Eigen::MatrixNf* AmBKt_data, Eigen::MatrixNMf* coeff_d2p_data);

enum tiny_ErrorCode tiny_InitPrimalCacheStretch(tiny_AdmmWorkspace* work, 
Eigen::MatrixMf* Quu_inv_data, Eigen::MatrixNf* AmBKt_data, Eigen::MatrixNMf* coeff_d2p_data);

enum tiny_ErrorCode tiny_ResetInfo(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_SetStateReference(tiny_AdmmWorkspace* work, Eigen::VectorNf* Xref);

enum tiny_ErrorCode tiny_SetInputReference(tiny_AdmmWorkspace* work, Eigen::VectorMf* Uref);

enum tiny_ErrorCode tiny_SetGoalState(tiny_AdmmWorkspace* work, Eigen::VectorNf* Xref,
Eigen::VectorNf* xg);

enum tiny_ErrorCode tiny_SetGoalInput(tiny_AdmmWorkspace* work, Eigen::VectorMf* Uref, Eigen::VectorMf* ug);

enum tiny_ErrorCode tiny_SetInitialState(tiny_AdmmWorkspace* work, Eigen::VectorNf* x0);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef AUXIL_H
