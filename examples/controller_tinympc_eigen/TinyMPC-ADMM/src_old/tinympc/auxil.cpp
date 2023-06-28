#include "auxil.h"

#ifdef __cplusplus
extern "C" {
#endif

enum tiny_ErrorCode tiny_InitSettings(tiny_AdmmSettings* stgs) {
  stgs->reg_min       = (float)REG_MIN;
  stgs->reg_max       = (float)REG_MAX;
  stgs->reg_mul       = (float)REG_MUL;
  stgs->en_reg_update = EN_REG_UPDATE;

  stgs->rho_init  = (float)RHO_INIT;
  stgs->rho_max   = (float)RHO_MAX;
  stgs->rho_mul   = (float)RHO_MUL;

  stgs->alpha_mul = (float)ALPHA_MUL;

  stgs->max_iter          = MAX_ITER;
  stgs->max_iter_riccati  = MAX_ITER_RICCATI;
  stgs->max_iter_ls       = MAX_ITER_LS;

  stgs->tol_abs_prim    = (float)TOL_ABS_PRIM;
  stgs->tol_abs_dual    = (float)TOL_ABS_DUAL;

  stgs->en_cstr_states  = EN_CSTR_STATES;
  stgs->en_cstr_inputs  = EN_CSTR_INPUTS;
  stgs->en_cstr_goal    = EN_CSTR_GOAL;

  stgs->verbose           = VERBOSE;
  stgs->adaptive_horizon  = ADAPTIVE_HORIZON;
  stgs->check_riccati     = CHECK_RICCATI;
  stgs->check_termination = CHECK_TERMINATION;
  stgs->warm_start        = WARM_START;
  stgs->time_limit        = TIME_LIMIT;

  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetUnconstrained(tiny_AdmmSettings* stgs) {
  stgs->en_cstr_states  = 0;
  stgs->en_cstr_inputs  = 0;
  stgs->en_cstr_goal    = 0;
  stgs->check_termination = 0;

  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitSolution(tiny_AdmmWorkspace* work,
Eigen::VectorNf* X, Eigen::VectorMf* U,
Eigen::VectorNf* YX, Eigen::VectorMf* YU, Eigen::VectorNf* YG,
Eigen::MatrixMNf* Kinf, Eigen::VectorMf* d, 
Eigen::MatrixNf* Pinf, Eigen::VectorNf* p) {
  work->soln->X     = X;  
  work->soln->U     = U;
  work->soln->YX    = YX;  
  work->soln->YU    = YU;
  work->soln->YG    = YG;
  work->soln->Kinf  = Kinf;
  work->soln->d     = d;
  work->soln->Pinf  = Pinf;
  work->soln->p     = p;
  return TINY_NO_ERROR;  
}

enum tiny_ErrorCode tiny_InitDataCost(tiny_AdmmWorkspace* work, 
Eigen::MatrixNf* Q, Eigen::VectorNf* q, Eigen::MatrixMf* R, Eigen::VectorMf* r, Eigen::VectorMf* r_tilde) {
  work->data->Q = Q;
  work->data->R = R;
  work->data->q = q;
  work->data->r = r;
  work->data->r_tilde = r_tilde;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitWorkspace(tiny_AdmmWorkspace* work,
                                       tiny_AdmmInfo* info,
                                       tiny_Model* model,
                                       tiny_AdmmData* data,
                                       tiny_AdmmSolution* soln,
                                       tiny_AdmmSettings* stgs) {
  work->data = data;
  work->info = info;
  work->soln = soln;
  work->stgs = stgs;
  work->data->model = model;

  // tiny_InitSolution(work);
  // tiny_InitData(work);

  // int n = model->nstates;
  int m = model->ninputs;
  int N = model->nhorizon;

  work->reg = (float)REG_MIN;
  work->alpha = (float)ALPHA;
  work->rho = work->stgs->rho_init;

  work->first_run = 1;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitWorkspaceTemp(tiny_AdmmWorkspace* work, Eigen::VectorMf* Qu,
Eigen::VectorMf* ZU, Eigen::VectorMf* ZU_new, Eigen::VectorNf* ZX, Eigen::VectorNf* ZX_new) {
  work->Qu     = Qu;
  work->ZU     = ZU;
  work->ZU_new = ZU_new;
  work->ZX     = ZX;
  work->ZX_new = ZX_new;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_InitPrimalCache(tiny_AdmmWorkspace* work, 
Eigen::MatrixMf* Quu_inv_data, Eigen::MatrixNf* AmBKt_data, Eigen::MatrixNMf* coeff_d2p_data) {
  work->Quu_inv   = Quu_inv_data;  
  work->AmBKt     = AmBKt_data; 
  work->coeff_d2p = coeff_d2p_data; 
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_ResetInfo(tiny_AdmmWorkspace* work) {
  work->info->iter = 0;
  work->info->iter_riccati = 0;
  work->info->status_val = TINY_UNSOLVED;

  work->info->obj_val = 0.0;
  work->info->pri_res = 0.0;
  work->info->dua_res = 0.0;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetStateReference(tiny_AdmmWorkspace* work, Eigen::VectorNf* Xref) {
  work->data->Xref = Xref;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetInputReference(tiny_AdmmWorkspace* work, Eigen::VectorMf* Uref) {
  work->data->Uref = Uref;
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetGoalState(tiny_AdmmWorkspace* work, Eigen::VectorNf* Xref,
Eigen::VectorNf* xg) {
  int N = work->data->model[0].nhorizon;
  work->data->Xref = Xref;
  for (int i = 0; i < N; ++i) {
    Xref[i] = *xg;
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetGoalInput(tiny_AdmmWorkspace* work, Eigen::VectorMf* Uref, Eigen::VectorMf* ug) {
  int N = work->data->model[0].nhorizon;
  work->data->Uref = Uref;
  for (int i = 0; i < N - 1; ++i) {
    Uref[i] = *ug;
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SetInitialState(tiny_AdmmWorkspace* work, Eigen::VectorNf* x0) {
  work->data->x0 = x0;
  return TINY_NO_ERROR;
}

#ifdef __cplusplus
}
#endif