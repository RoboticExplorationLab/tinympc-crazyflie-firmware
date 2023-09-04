#include "constraint_linear.h"

#ifdef __cplusplus
extern "C" {
#endif

enum tiny_ErrorCode tiny_SetInputBound(tiny_AdmmWorkspace* work, Eigen::MatrixMf* Acu, Eigen::VectorMf* lcu, Eigen::VectorMf* ucu) {
  work->stgs->en_cstr_inputs = 1;
  work->data->Acu = Acu;
  (*(work->data->Acu)).setIdentity();
  work->data->lcu = lcu;
  work->data->ucu = ucu;
  return TINY_NO_ERROR;
}

// enum tiny_ErrorCode tiny_SetStateBound(tiny_AdmmWorkspace* work, Eigen::MatrixNf* Acx, Eigen::VectorNf* lcx, Eigen::VectorNf* ucx) {
//   work->stgs->en_cstr_states = 1;
//   work->data->Acx = Acx;
//   (*(work->data->Acx)).setIdentity();
//   work->data->lcx = lcx;
//   work->data->ucx = ucx;
//   return TINY_NO_ERROR;
// }

enum tiny_ErrorCode tiny_SetStateConstraint(tiny_AdmmWorkspace* work, Eigen::VectorNf* Acx, Eigen::VectorNf* lcx, Eigen::VectorNf* ucx) {
  work->stgs->en_cstr_states = 1;
  work->data->Acx = Acx;
  work->data->lcx = lcx;
  work->data->ucx = ucx;
  return TINY_NO_ERROR;
}

// enum tiny_ErrorCode tiny_ProjectInput(tiny_AdmmWorkspace* work) {
//   int n = work->data->model[0].ninputs;
//   int N = work->data->model[0].ninputs;

//   for (int k = 0; k < N - 1; ++k) {
//     for (int i = 0; i < n; ++i) {

//       work->ZU_new[k].data[i] = T_MIN(T_MAX(z[i],
//                                 work->data->lcu[i]),  // Between lower
//                                 work->data->ucu[i]);  // and upper bounds
//     } 
//   }
//   return TINY_NO_ERROR;
// }

int IsConstrained(tiny_AdmmWorkspace* work) {
  if (!work->stgs->en_cstr_goal && 
      !work->stgs->en_cstr_inputs && 
      !work->stgs->en_cstr_states) {
    return 0; // unconstrained
  }
  return 1;    
}

#ifdef __cplusplus
}
#endif