#ifndef AUXIL_H
# define AUXIL_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include "types.h"
#include "utils.h"

enum tiny_ErrorCode tiny_InitSettings(tiny_AdmmSettings* stgs);

enum tiny_ErrorCode tiny_SetUnconstrained(tiny_AdmmSettings* stgs);

// enum tiny_ErrorCode tiny_InitData(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_InitDataQuadCostFromArray(tiny_AdmmWorkspace* work, 
sfloat* Q_data, sfloat* R_data);

enum tiny_ErrorCode tiny_InitDataLinearCostFromArray(tiny_AdmmWorkspace* work, 
Matrix* q, Matrix* r, Matrix* r_tilde, sfloat* q_data, sfloat* r_data, sfloat* r_tilde_data);

enum tiny_ErrorCode tiny_InitSolnTrajFromArray(tiny_AdmmWorkspace* work,
Matrix* X, Matrix* U,
sfloat* X_data, sfloat* U_data);

enum tiny_ErrorCode tiny_InitSolnDualsFromArray(tiny_AdmmWorkspace* work,
Matrix* YX, Matrix* YU,
sfloat* YX_data, sfloat* YU_data, sfloat* YG_data);

enum tiny_ErrorCode tiny_InitSolnGainsFromArray(tiny_AdmmWorkspace* work, 
Matrix* d, Matrix* p, sfloat* d_data, sfloat* p_data, 
sfloat* Kinf_data, sfloat* Pinf_data);

enum tiny_ErrorCode tiny_InitWorkspace(tiny_AdmmWorkspace* work,
                                       tiny_AdmmInfo* info,
                                       tiny_Model* model,
                                       tiny_AdmmData* data,
                                       tiny_AdmmSolution* soln,
                                       tiny_AdmmSettings* stgs);

enum tiny_ErrorCode tiny_InitWorkspaceTempData(tiny_AdmmWorkspace* work, 
Matrix* ZU, Matrix* ZU_new, Matrix* ZX, Matrix* ZX_new, sfloat* temp_data);

// enum tiny_ErrorCode tiny_EvalPrimalCache(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_InitPrimalCache(tiny_AdmmWorkspace* work, 
sfloat* Quu_inv_data, sfloat* AmBKt_data, sfloat* coeff_d2p_data);

enum tiny_ErrorCode tiny_ResetInfo(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_SetStateReference(tiny_AdmmWorkspace* work, Matrix* Xref, 
sfloat* Xref_data);

enum tiny_ErrorCode tiny_SetInputReference(tiny_AdmmWorkspace* work, Matrix* Uref,
sfloat* Uref_data);

enum tiny_ErrorCode tiny_SetReference(tiny_AdmmWorkspace* work, Matrix* Xref, 
Matrix* Uref, sfloat* Xref_data, sfloat* Uref_data);

enum tiny_ErrorCode tiny_SetGoalReference(tiny_AdmmWorkspace* work, Matrix* Xref,
Matrix* Uref, sfloat* xg_data, sfloat* ug_data);

enum tiny_ErrorCode tiny_SetInitialState(tiny_AdmmWorkspace* work, sfloat* x0_data);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef AUXIL_H
