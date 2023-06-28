#include "model.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

enum tiny_ErrorCode tiny_InitModel(tiny_Model* model, const int nstates,
                                   const int ninputs, const int nhorizon,
                                   const int ltv, const int affine, 
                                   const float dt,
                                   Eigen::MatrixNf* A, 
                                   Eigen::MatrixNMf* B, Eigen::VectorNf* f) {

  model->nstates  = nstates;
  model->ninputs  = ninputs;
  model->nhorizon = nhorizon;

  model->ltv      = ltv;
  model->affine   = affine;
  model->dt       = dt;

  model->A = TINY_NULL;
  model->B = TINY_NULL;
  model->f = TINY_NULL;
  model->get_jacobians = TINY_NULL;
  model->get_nonl_model = TINY_NULL; 

  model->A = A;
  model->B = B;
  model->f = f;
  return TINY_NO_ERROR;
}

// enum tiny_ErrorCode tiny_SetModelJacFunc(
//     tiny_Model* model, 
//     void (*get_jacobians)(Eigen::MatrixXf*, Eigen::MatrixXf*, Eigen::VectorXf*, Eigen::VectorXf*)) {
//   model->get_jacobians = get_jacobians;
//   return TINY_NO_ERROR;      
// }

// enum tiny_ErrorCode tiny_SetModelNonlFunc(
//     tiny_Model* model, 
//     void (*get_nonl_model)(Eigen::VectorXf*, Eigen::VectorXf*, Eigen::VectorXf*)) {
//   model->get_nonl_model = get_nonl_model;
//   return TINY_NO_ERROR;    
// }

enum tiny_ErrorCode tiny_EvalModel(Eigen::VectorNf* xn, Eigen::VectorNf* x, Eigen::VectorMf* u, tiny_Model* model, const int k) {
  if (model->affine) {
    return TINY_NOT_SUPPORTED;
  }
  else {
    (*xn).noalias() = (model->A[k]).lazyProduct(*x) + (model->B[k]).lazyProduct(*u);
  } 
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_RollOutClosedLoop(tiny_AdmmWorkspace* work) {
  tiny_Model* model = work->data->model;
  tiny_Model* model_s = work->data->model_s;
  int N = model[0].nhorizon;
  int adaptive_horizon = work->stgs->adaptive_horizon;
  
  if (model[0].ltv) { 
    return TINY_NOT_SUPPORTED;
  }
  else {
    for (int k = 0; k < N - 1; ++k) {
      // Control input: u = - d - K*x
      (work->soln->U[k]) = -work->soln->d[k];
      // Next state: x = A*x + B*u + f
      if (adaptive_horizon > 0 && k > adaptive_horizon - 1) {
        (work->soln->U[k]).noalias() -= (*(work->soln->Kinf_s)).lazyProduct(work->soln->X[k]);
        tiny_EvalModel(&(work->soln->X[k + 1]), &(work->soln->X[k]), &(work->soln->U[k]), &model_s[0], 0);
      }
      else {
        (work->soln->U[k]).noalias() -= (*(work->soln->Kinf)).lazyProduct(work->soln->X[k]);
        tiny_EvalModel(&(work->soln->X[k + 1]), &(work->soln->X[k]), &(work->soln->U[k]), &model[0], 0);
      }
    }        
  }
  return TINY_NO_ERROR;
}

enum tiny_ErrorCode tiny_RollOutOpenLoop(tiny_AdmmWorkspace* work) {
  tiny_Model* model = work->data->model;
  tiny_Model* model_s = work->data->model_s;
  int N = model[0].nhorizon;
  int adaptive_horizon = work->stgs->adaptive_horizon;
  
  if (model[0].ltv) {
    for (int k = 0; k < N - 1; ++k) {
      // Next state: x = A*x + B*u + f
      if (adaptive_horizon > 0 && k > adaptive_horizon - 1) {
        tiny_EvalModel(&(work->soln->X[k + 1]), &(work->soln->X[k]), &(work->soln->U[k]), &model_s[0], k);
      }
      else {
        tiny_EvalModel(&(work->soln->X[k + 1]), &(work->soln->X[k]), &(work->soln->U[k]), &model[0], k);
      }
    }    
  }
  else {
    for (int k = 0; k < N - 1; ++k) {
      // Next state: x = A*x + B*u + f
      if (adaptive_horizon > 0 && k > adaptive_horizon - 1) {
        tiny_EvalModel(&(work->soln->X[k + 1]), &(work->soln->X[k]), &(work->soln->U[k]), &model_s[0], 0);
      }
      else {
        tiny_EvalModel(&(work->soln->X[k + 1]), &(work->soln->X[k]), &(work->soln->U[k]), &model[0], 0);
      }
    }        
  }
  return TINY_NO_ERROR;
}

// enum tiny_ErrorCode tiny_UpdateModelJac(tiny_AdmmWorkspace* work) {
//   tiny_Model* model = work->data->model;
//   int N = model[0].nhorizon;
//   // LTV model
//   if (model[0].ltv) {
//     for (int i = 0; i < N - 1; ++i) {
//       // get A and B
//       model[0].get_jacobians(&(model[0].A[i]), 
//                              &(model[0].B[i]), 
//                              &(work->data->Xref[i]), 
//                              &(work->data->Uref[i]));
//       if (model[0].affine) {
//         // get f = x1 - Ax - Bu
//         if (model[0].get_nonl_model != TINY_NULL) {
//           model[0].get_nonl_model(&(model[0].f[i]),
//                                   &(work->data->Xref[i]), 
//                                   &(work->data->Uref[i]));
//           // slap_MatMulAdd(model[0].f[i], 
//           //                model[0].A[i], 
//           //                work->data->Xref[i], -1, 1);
//           // slap_MatMulAdd(model[0].f[i], 
//           //                model[0].B[i], 
//           //                work->data->Uref[i], -1, 1);
//           model[0].f[i] = -(model[0].A[i]).lazyProduct(work->data->Xref[i]) 
//                           -(model[0].B[i]).lazyProduct(work->data->Uref[i]);
//         }
//       }
//     }
//   }
//   // LTI model
//   else {
//     // get A and B
//     model[0].get_jacobians(&(model[0].A[0]), 
//                            &(model[0].B[0]), 
//                            &(work->data->Xref[N-1]),
//                            &(work->data->Uref[N-1]));
//     if (model[0].affine) {                     
//       // get f = x1 - Ax - Bu
//       if (model[0].get_nonl_model != TINY_NULL) {
//         model[0].get_nonl_model(model[0].f, 
//                                 &(work->data->Xref[N-1]),
//                                 &(work->data->Uref[N-1]));
//         // slap_MatMulAdd(model[0].f[0], 
//         //                model[0].A[0], 
//         //                work->data->Xref[N-1], -1, 1);
//         // slap_MatMulAdd(model[0].f[0], 
//         //                model[0].B[0], 
//         //                work->data->Uref[N-1], -1, 1);
//         model[0].f[0] = -(model[0].A[0]).lazyProduct(work->data->Xref[N-1]) 
//                         -(model[0].B[0]).lazyProduct(work->data->Uref[N-1]); 
//       }
//     }
//   }
//   return TINY_NO_ERROR;
// }

// enum tiny_ErrorCode tiny_UpdateModelJacAbout(tiny_AdmmWorkspace* work,
//                                              Eigen::VectorXf* X, Eigen::VectorXf* U) {
//   tiny_Model* model = work->data->model;
//   int N = model[0].nhorizon;
//   // LTV model
//   if (model[0].ltv) {
//     for (int i = 0; i < N - 1; ++i) {
//       // get A and B
//       model[0].get_jacobians(&(model[0].A[i]), &(model[0].B[i]), X[i], U[i]);
      
//       if (model[0].affine) {
//         // get f = x1 - Ax - Bu
//         if (model[0].get_nonl_model != TINY_NULL) {
//           model[0].get_nonl_model(&(model[0].f[i]), &X[i], &U[i]);
//           slap_MatMulAdd(model[0].f[i], model[0].A[i], X[i], -1, 1);
//           slap_MatMulAdd(model[0].f[i], model[0].B[i], U[i], -1, 1);
//         }
//       }
//     }
//   }
//   // LTI model
//   else {
//     // get A and B
//     model[0].get_jacobians(&(model[0].A[0]), &(model[0].B[0]), X[N-1], U[N-1]);
//     if (model[0].affine) {      
//       printf("A");               
//       // get f = x1 - Ax - Bu
//       if (model[0].get_nonl_model != TINY_NULL) {
//         model[0].get_nonl_model(model[0].f, 
//                                 work->data->Xref[N-1],
//                                 work->data->Uref[N-1]);
//         slap_MatMulAdd(model[0].f[0], model[0].A[0], X[N-1], -1, 1);
//         slap_MatMulAdd(model[0].f[0], model[0].B[0], U[N-1], -1, 1);
//       }
//     }
//   }
//   return TINY_NO_ERROR;
// }

# ifdef __cplusplus
}
# endif // ifdef __cplusplus