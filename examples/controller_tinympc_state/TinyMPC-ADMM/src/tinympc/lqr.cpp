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
  tiny_Model* model_s = work->data->model_s;
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
    for (int k = N - 2; k >= 0; --k) {
      if (work->stgs->adaptive_horizon > 0 && k >= work->stgs->adaptive_horizon) {
        // printf("Stretch\n");
        /* Compute  Qu = B'*p[k+1] + r[k] */
        (*(work->Qu)).noalias() = (model_s[0].B[0]).transpose().lazyProduct(work->soln->p[k+1]);
        (*(work->Qu)) += work->data->r_tilde[k];
        /* Compute d = Quu\Qu */
        (work->soln->d[k]).noalias() = (*(work->Quu_inv_s)).lazyProduct(*(work->Qu));
        // PrintMatrixT(work->soln->d[k]);

        /* Compute p[k] .= q[k] + AmBKt*p[k+1] - Kinf'*r[k] + coeff_d2p*d[k] */
        work->soln->p[k] = work->data->q[k];
        (work->soln->p[k]).noalias() += (*(work->AmBKt_s)).lazyProduct(work->soln->p[k+1])
                          - ((*(work->soln->Kinf_s)).transpose()).lazyProduct(work->data->r_tilde[k]) + (*(work->coeff_d2p_s)).lazyProduct(work->soln->d[k]);   
      }
      else {
        // printf("Normal\n");
        /* Compute  Qu = B'*p[k+1] + r[k] */
        (*(work->Qu)).noalias() = (model[0].B[0]).transpose().lazyProduct(work->soln->p[k+1]);
        (*(work->Qu)) += work->data->r_tilde[k];

        /* Compute d = Quu\Qu */
        (work->soln->d[k]).noalias() = (*(work->Quu_inv)).lazyProduct(*(work->Qu));
        // PrintMatrixT(work->soln->d[k]);

        /* Compute p[k] .= q[k] + AmBKt*p[k+1] - Kinf'*r[k] + coeff_d2p*d[k] */
        work->soln->p[k] = work->data->q[k];
        (work->soln->p[k]).noalias() += (*(work->AmBKt)).lazyProduct(work->soln->p[k+1])
                          - ((*(work->soln->Kinf)).transpose()).lazyProduct(work->data->r_tilde[k]) + (*(work->coeff_d2p)).lazyProduct(work->soln->d[k]);   
      }
    }
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_SolveLqr(tiny_AdmmWorkspace* work) {
  work->soln->X[0] = *(work->data->x0);
  tiny_BackwardPassGrad(work);  
  // printf("Done BackwardPassGrad()\n");
  tiny_ForwardPass(work);  
  // printf("Done ForwardPass()\n");
  return TINY_NO_ERROR;
}

# ifdef __cplusplus
}
# endif // ifdef __cplusplus