#include "lqr.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

enum tiny_ErrorCode tiny_ForwardPass(tiny_AdmmWorkspace* work) {
  tiny_RollOutClosedLoop(work);
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_BackwardPassGrad(tiny_AdmmWorkspace* work) {
  tiny_Model* model = work->data->model;
  int N = model[0].nhorizon;

  if (model[0].ltv && model[0].affine) {
    return TINY_NOT_SUPPORTED;
  }
  if (model[0].ltv && !model[0].affine) {
    return TINY_NOT_SUPPORTED;
  }
  if (!model[0].ltv && model[0].affine) {
    return TINY_NOT_SUPPORTED;
  }
  // LTI model
  if (!model[0].ltv && !model[0].affine) {
    // printf("backward pass\n");
    for (int k = N - 2; k >= 0; --k) {
      /* Compute  Qu = B'*p[k+1] + r[k] */
      // slap_MatMulAtB(work->Qu, model[0].B[0], work->soln->p[k+1]);
      // MatAdd(work->Qu, work->Qu, work->data->r_tilde[k], 1);
      (*(work->Qu)).noalias() = (model[0].B[0]).transpose().lazyProduct(work->soln->p[k+1]);
      (*(work->Qu)) += work->data->r_tilde[k];

      /* Compute d = Quu\Qu */
      // slap_MatMulAB(work->soln->d[k], work->Quu_inv, work->Qu);
      // (work->soln->d[k]).noalias() = work->Quu_inv * work->Qu;
      (work->soln->d[k]).noalias() = (*(work->Quu_inv)).lazyProduct(*(work->Qu));
      // PrintMatrixT(work->soln->d[k]);

      /* Compute p[k] .= q[k] + AmBKt*p[k+1] - Kinf'*r[k] + coeff_d2p*d[k] */
      // slap_MatMulAtB(work->soln->p[k], work->soln->Kinf, work->data->r_tilde[k]);
      // MatMulAdd(work->soln->p[k], work->coeff_d2p, work->soln->d[k], 1, -1);
      // MatMulAdd(work->soln->p[k], work->AmBKt, work->soln->p[k+1], 1, 1);
      // MatAdd(work->soln->p[k],work->soln->p[k], work->data->q[k], 1);  
      work->soln->p[k] = work->data->q_tilde[k];
      (work->soln->p[k]).noalias() += (*(work->AmBKt)).lazyProduct(work->soln->p[k+1])
                         - ((*(work->soln->Kinf)).transpose()).lazyProduct(work->data->r_tilde[k]) + (*(work->coeff_d2p)).lazyProduct(work->soln->d[k]);   
      // PrintMatrixT(work->soln->p[k]);
    }
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SolveLqr(tiny_AdmmWorkspace* work) {
  work->soln->X[0] = *(work->data->x0);
  tiny_BackwardPassGrad(work);  
  tiny_ForwardPass(work);  
  return TINY_NO_ERROR;
}

# ifdef __cplusplus
}
# endif // ifdef __cplusplus