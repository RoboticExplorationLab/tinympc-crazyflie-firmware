// MPC
// Scenerio: make Crazyflie hovering
//

#include "quadrotor.h"
#include "slap/slap.h"
#include "time.h"
#include "tinympc/tinympc.h"

// Macro variables
#define H 0.02       // dt
#define NSTATES 12   // no. of states (error state)
#define NINPUTS 4    // no. of controls
#define NHORIZON 5  // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSIM 100     // simulation steps (fixed with reference data)

int main() {
  /* Start MPC initialization*/

  // Create data array 
  sfloat x0_data[NSTATES] = {0, 0, 0, 1, 0, 0,
                             0, 0, 0, 0, 0, 0};  // initial state
  sfloat xg_data[NSTATES] = {0};  
  sfloat ug_data[NINPUTS] = {0};      // goal input if needed
  sfloat Xhrz_data[NSTATES * NHORIZON] = {0};      // save X for one horizon
  sfloat X_data[NSTATES * NSIM] = {0};             // save X for the whole run
  sfloat Uhrz_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat A_data[NSTATES*NSTATES]= {
    1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.000000f,-0.003849f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,-0.392400f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.003849f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.392400f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
    0.000000f,0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
    0.000000f,-0.000013f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,-0.001999f,0.000000f,1.000000f,0.000000f,0.000000f,
    0.000013f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.001999f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
    0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,
  };
  sfloat B_data[NSTATES*NINPUTS]= {
    0.000019f,0.000019f,0.000962f,-0.029620f,0.030207f,-0.004066f,0.004101f,0.004021f,0.096236f,-5.923901f,6.041486f,-0.813298f,
    -0.000018f,0.000017f,0.000962f,-0.027140f,-0.027503f,0.005295f,-0.003734f,0.003685f,0.096236f,-5.428009f,-5.500535f,1.059020f,
    -0.000019f,-0.000019f,0.000962f,0.029845f,-0.029577f,-0.001855f,-0.004016f,-0.004052f,0.096236f,5.968959f,-5.915321f,-0.370998f,
    0.000017f,-0.000017f,0.000962f,0.026915f,0.026872f,0.000626f,0.003648f,-0.003654f,0.096236f,5.382950f,5.374370f,0.125275f,
  };
  sfloat f_data[NSTATES] = {0};
  sfloat Kinf_data[NINPUTS*NSTATES] = {
    0.074926f,-0.087307f,-0.170980f,0.183361f,
    0.142503f,0.116560f,-0.104167f,-0.154896f,
    1.397135f,1.397135f,1.397135f,1.397135f,
    -0.653577f,-0.529393f,0.472648f,0.710322f,
    0.334850f,-0.391543f,-0.788093f,0.844787f,
    -0.272398f,0.270565f,-0.275297f,0.277130f,
    0.055876f,-0.065199f,-0.129091f,0.138415f,
    0.107376f,0.087484f,-0.078152f,-0.116708f,
    0.583418f,0.583418f,0.583418f,0.583418f,
    -0.051908f,-0.041623f,0.037125f,0.056407f,
    0.025881f,-0.030376f,-0.062909f,0.067404f,
    -0.287018f,0.285076f,-0.290085f,0.292027f,
  };
  sfloat Pinf_data[NSTATES*NSTATES] = {
    376.096473f,-0.013921f,0.000000f,0.061321f,338.072148f,0.081573f,112.687691f,-0.010361f,0.000000f,0.004544f,1.647141f,0.084648f,
    -0.013921f,376.082931f,-0.000000f,-338.012160f,-0.061321f,-0.032658f,-0.010361f,112.677580f,0.000000f,-1.642698f,-0.004544f,-0.033889f,
    -0.000000f,-0.000000f,208.790813f,0.000000f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,16.505696f,0.000000f,-0.000000f,-0.000000f,
    0.061321f,-338.012160f,0.000000f,1446.515966f,0.277569f,0.153801f,0.046110f,-247.529765f,0.000000f,7.178445f,0.021058f,0.159773f,
    338.072148f,-0.061321f,-0.000000f,0.277569f,1446.790778f,0.384124f,247.575126f,-0.046110f,-0.000000f,0.021058f,7.199401f,0.399039f,
    0.081573f,-0.032658f,-0.000000f,0.153801f,0.384124f,97.779835f,0.062145f,-0.024881f,-0.000000f,0.012386f,0.030932f,0.949901f,
    112.687691f,-0.010361f,-0.000000f,0.046110f,247.575126f,0.062145f,66.874239f,-0.007743f,-0.000000f,0.003439f,1.217014f,0.064496f,
    -0.010361f,112.677580f,-0.000000f,-247.529765f,-0.046110f,-0.024881f,-0.007743f,66.866651f,0.000000f,-1.213627f,-0.003439f,-0.025822f,
    0.000000f,-0.000000f,16.505696f,0.000000f,0.000000f,-0.000000f,0.000000f,-0.000000f,7.227418f,0.000000f,-0.000000f,-0.000000f,
    0.004544f,-1.642698f,0.000000f,7.178445f,0.021058f,0.012386f,0.003439f,-1.213627f,0.000000f,1.043715f,0.001687f,0.013041f,
    1.647141f,-0.004544f,-0.000000f,0.021058f,7.199401f,0.030932f,1.217014f,-0.003439f,-0.000000f,0.001687f,1.045421f,0.032567f,
    0.084648f,-0.033889f,-0.000000f,0.159773f,0.399039f,0.949901f,0.064496f,-0.025822f,-0.000000f,0.013041f,0.032567f,1.495774f,
  };
  sfloat Quu_inv_data[NINPUTS*NINPUTS] = {
    0.172741f,0.050332f,0.167886f,0.047495f,
    0.050332f,0.171730f,0.049139f,0.167253f,
    0.167886f,0.049139f,0.175090f,0.046339f,
    0.047495f,0.167253f,0.046339f,0.177367f,
  };
  sfloat AmBKt_data[NSTATES*NSTATES] = {
    0.999991f,-0.000000f,-0.000000f,0.000000f,0.003807f,0.000000f,0.019993f,-0.000000f,-0.000000f,0.000000f,0.000009f,0.000000f,
    -0.000000f,0.999991f,-0.000000f,-0.003806f,-0.000000f,-0.000000f,-0.000000f,0.019993f,-0.000000f,-0.000009f,-0.000000f,-0.000000f,
    -0.000000f,-0.000000f,0.994622f,0.000000f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.017754f,0.000000f,-0.000000f,-0.000000f,
    0.000018f,0.014662f,0.000000f,0.933049f,0.000075f,0.000032f,0.000013f,0.011028f,0.000000f,0.004707f,0.000006f,0.000033f,
    -0.014649f,-0.000018f,-0.000000f,0.000075f,0.933106f,0.000080f,-0.011019f,-0.000013f,0.000000f,0.000006f,0.004711f,0.000083f,
    0.000335f,-0.000134f,-0.000000f,0.000577f,0.001444f,0.996775f,0.000246f,-0.000098f,-0.000000f,0.000043f,0.000107f,0.006602f,
    -0.001989f,-0.000002f,-0.000000f,0.000010f,0.383318f,0.000011f,0.998504f,-0.000002f,-0.000000f,0.000001f,0.001281f,0.000011f,
    -0.000002f,-0.001991f,-0.000000f,-0.383310f,-0.000010f,-0.000004f,-0.000002f,0.998503f,-0.000000f,-0.001281f,-0.000001f,-0.000005f,
    -0.000000f,-0.000000f,-0.537819f,0.000000f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.775417f,0.000000f,-0.000000f,-0.000000f,
    0.003504f,2.932430f,0.000000f,-13.390115f,0.014964f,0.006424f,0.002565f,2.205670f,0.000000f,-0.058661f,0.001107f,0.006666f,
    -2.929753f,-0.003504f,-0.000000f,0.014963f,-13.378707f,0.016071f,-2.203712f,-0.002565f,-0.000000f,0.001107f,-0.057819f,0.016676f,
    0.066993f,-0.026783f,-0.000000f,0.115450f,0.288773f,-0.644926f,0.049258f,-0.019693f,-0.000000f,0.008569f,0.021434f,0.320463f,
  };
  sfloat coeff_d2p_data[NSTATES*NINPUTS] = {
    0.074927f,0.142503f,1.397135f,-0.653575f,0.334852f,-0.271611f,0.055876f,0.107376f,0.583418f,-0.051908f,0.025881f,-0.287010f,
    -0.087308f,0.116561f,1.397135f,-0.529393f,-0.391547f,0.269611f,-0.065200f,0.087485f,0.583418f,-0.041623f,-0.030376f,0.285066f,
    -0.170980f,-0.104168f,1.397135f,0.472648f,-0.788090f,-0.274808f,-0.129091f,-0.078153f,0.583418f,0.037125f,-0.062909f,-0.290080f,
    0.183361f,-0.154896f,1.397135f,0.710321f,0.844785f,0.276807f,0.138414f,-0.116708f,0.583418f,0.056407f,0.067403f,0.292024f,
  };

  sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat p_data[NSTATES * NHORIZON] = {0};
  sfloat Q_data[NSTATES * NSTATES] = {0};
  sfloat R_data[NINPUTS * NINPUTS] = {0};
  sfloat q_data[NSTATES*(NHORIZON-1)] = {0};
  sfloat r_data[NINPUTS*(NHORIZON-1)] = {0};
  sfloat r_tilde_data[NINPUTS*(NHORIZON-1)] = {0};

  sfloat umin_data[NINPUTS] = {0};
  sfloat umax_data[NINPUTS] = {0};
  // Put constraints on u, x
  sfloat Acu_data[NINPUTS * NINPUTS] = {0};  
  sfloat YU_data[NINPUTS * (NHORIZON - 1)] = {0};

  // Created matrices
  Matrix X[NSIM];
  Matrix Xref[NSIM];
  Matrix Uref[NSIM - 1];
  Matrix Xhrz[NHORIZON];
  Matrix Uhrz[NHORIZON - 1];
  Matrix d[NHORIZON - 1];
  Matrix p[NHORIZON];
  Matrix YU[NHORIZON - 1];
  Matrix ZU[NHORIZON - 1];
  Matrix ZU_new[NHORIZON - 1];
  Matrix q[NHORIZON-1];
  Matrix r[NHORIZON-1];
  Matrix r_tilde[NHORIZON-1];
  Matrix A;
  Matrix B;
  Matrix f;

  for (int i = 0; i < NSIM; ++i) {
    X[i] = slap_MatrixFromArray(NSTATES, 1, &X_data[i * NSTATES]);
  }

  /* Create TinyMPC struct and problem data*/
  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, 0.02);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);
  stgs.rho_init = 1e0;  // Important (select offline, associated with precomp.)

  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  sfloat temp_data[work.data_size];
  T_INIT_ZEROS(temp_data);
  tiny_InitWorkspaceTempData(&work, ZU, ZU_new, 0, 0, temp_data);
  tiny_InitPrimalCache(&work, Quu_inv_data, AmBKt_data, coeff_d2p_data);

  tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);
  tiny_InitSolnTrajFromArray(&work, Xhrz, Uhrz, Xhrz_data, Uhrz_data);
  tiny_InitSolnGainsFromArray(&work, d, p, d_data, p_data, Kinf_data, Pinf_data);
  tiny_InitSolnDualsFromArray(&work, 0, YU, 0, YU_data, 0);

  tiny_SetInitialState(&work, x0_data);  
  tiny_SetGoalReference(&work, Xref, Uref, xg_data, ug_data);

  /* Set up LQR cost */
  tiny_InitDataQuadCostFromArray(&work, Q_data, R_data);
  // slap_SetIdentity(prob.Q, 1000e-1);
  sfloat Qdiag[NSTATES] = {10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  slap_SetDiagonal(data.Q, Qdiag, NSTATES);
  slap_SetIdentity(data.R, 1);
  slap_AddIdentity(data.R, work.rho); // \tilde{R}
  tiny_InitDataLinearCostFromArray(&work, q, r, r_tilde, q_data, r_data, r_tilde_data);

  /* Set up constraints */
  tiny_SetInputBound(&work, Acu_data, umin_data, umax_data);
  slap_SetConst(data.ucu, 0.5);
  slap_SetConst(data.lcu, -0.5);

  tiny_UpdateLinearCost(&work);

  if (0) {
    printf("\nProblem Info: \n");
    PrintMatrix(work.data->model->A[0]);
    PrintMatrix(work.data->model->B[0]);
    PrintMatrix(work.data->Q);
    PrintMatrix(work.data->R);
    PrintMatrixT(work.data->x0);
    PrintMatrixT(work.data->Xref[NHORIZON-5]);
    PrintMatrixT(work.data->Uref[NHORIZON-5]);
    PrintMatrixT(work.data->q[NHORIZON-5]);
    PrintMatrixT(work.data->r[NHORIZON-5]);
  }

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 1000;           // limit this if needed
  stgs.verbose = 1;
  stgs.check_termination = 1;
  stgs.tol_abs_dual = 1e-2;
  stgs.tol_abs_prim = 1e-2;

  // Absolute formulation:
  // Warm-starting since horizon data is reused
  // Stop earlier as horizon exceeds the end
  MatCpy(X[0], work.data->x0);  
  srand(1);  // random seed

  /* End of MPC initialization*/

  /* Start MPC loop */

  for (int k = 0; k < NSIM - NHORIZON - 1; ++k) {
    Matrix pose = slap_CreateSubMatrix(X[k], 0, 0, 6, 1);
    Matrix pose_ref = slap_CreateSubMatrix(Xref[0], 0, 0, 6, 1);
    // printf("ex[%d] = %.4f\n", k, slap_NormedDifference(X[k], Xref[0]));
    printf("ex[%d] =  %.4f\n", k, slap_NormedDifference(pose, pose_ref));
    // printf("%.4f\n", slap_NormedDifference(pose, pose_ref));

    // Inject noise into measurement
    for (int j = 0; j < NSTATES; ++j) {
      X[k].data[j] += X[k].data[j] * T_NOISE(5);
    }

    clock_t start, end;
    double cpu_time_used;
    start = clock();

    MatCpy(work.data->x0, X[k]);  // update current measurement

    // Warm-start by previous solution
    tiny_ShiftFill(Uhrz, T_ARRAY_SIZE(Uhrz));

    // Solve optimization problem using Augmented Lagrangian TVLQR
    tiny_SolveAdmm(&work);

    end = clock();
    cpu_time_used = ((double)(end - start)) * 1000 / CLOCKS_PER_SEC;  // ms
    printf("solve time:        %f\n", cpu_time_used);
    // printf("%f\n", cpu_time_used);

    if(work.info->status_val != TINY_SOLVED) {
      printf("!!! STOP AS SOLVER FAILED !!!\n");
      return 0;
    }

    // PrintMatrixT(Uhrz[0]);

    // Matrix pos = slap_CreateSubMatrix(X[k], 0, 0, 3, 1);
    // PrintMatrixT(pos);

    // === 2. Simulate dynamics using the first control solution ===
    // tiny_QuadNonlinearDynamics(&X[k + 1], X[k], Uref[k]);
    // tiny_Clamp(ZU_new[0].data, umin_data[0], umax_data[0], NINPUTS);
    tiny_QuadNonlinearDynamics(&X[k + 1], X[k], ZU_new[0]);
    // tiny_DynamicsLti(&X[k + 1], X[k], Uref[k], model);
  }

  return 0;
}