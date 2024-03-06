#include <iostream>

#include "admm.hpp"

#define DEBUG_MODULE "TINYALG"

extern "C"
{

    static uint64_t startTimestamp;

    void reset_dual(TinySolver *solver) {

        // Reset box constraint duals
        solver->work->bounds->g = tiny_MatrixNxNh::Zero(); // states
        solver->work->bounds->y = tiny_MatrixNuNhm1::Zero(); // inputs

        // Reset second order cone constraint duals
        if (solver->settings->en_state_soc) {
            for (int i=0; i<NUM_STATE_CONES; i++) {
                solver->work->socs->gc[i] = tiny_MatrixNxNh::Zero(); // states
            }
        }

        if (solver->settings->en_input_soc) {
            for (int i=0; i<NUM_INPUT_CONES; i++) {
                solver->work->socs->yc[i] = tiny_MatrixNuNhm1::Zero(); // inputs
            }
        }
    }

    void reset_problem(TinySolver *solver) {
        // Reference trajectory for one horizon
        solver->work->Xref = tiny_MatrixNxNh::Zero();
        solver->work->Uref = tiny_MatrixNuNhm1::Zero();

        // State and input
        solver->work->x = tiny_MatrixNxNh::Zero();
        solver->work->u = tiny_MatrixNuNhm1::Zero();

        // Linear control cost terms
        solver->work->q = tiny_MatrixNxNh::Zero();
        solver->work->r = tiny_MatrixNuNhm1::Zero();
        
        // Linear Riccati backward pass terms
        solver->work->p = tiny_MatrixNxNh::Zero();
        solver->work->d = tiny_MatrixNuNhm1::Zero();
        
        // Set state constraint slack and dual variables to zero
        solver->work->bounds->v = tiny_MatrixNxNh::Zero();
        solver->work->bounds->vnew = tiny_MatrixNxNh::Zero();
        solver->work->bounds->g = tiny_MatrixNxNh::Zero();
        for (int i=0; i<NUM_STATE_CONES; i++) {
            solver->work->socs->vc[i] = tiny_MatrixNxNh::Zero();
            solver->work->socs->vcnew[i] = tiny_MatrixNxNh::Zero();
            solver->work->socs->gc[i] = tiny_MatrixNxNh::Zero();
        }

        // Set input constraint slack and dual variables to zero
        solver->work->bounds->z = tiny_MatrixNuNhm1::Zero();
        solver->work->bounds->znew = tiny_MatrixNuNhm1::Zero();
        solver->work->bounds->y = tiny_MatrixNuNhm1::Zero();
        for (int i=0; i<NUM_INPUT_CONES; i++) {
            solver->work->socs->zc[i] = tiny_MatrixNuNhm1::Zero();
            solver->work->socs->zcnew[i] = tiny_MatrixNuNhm1::Zero();
            solver->work->socs->yc[i] = tiny_MatrixNuNhm1::Zero();
        }

        // Iteration tracking values
        solver->work->status = 0;
        solver->work->iter = 0;
    }

    

    /**
     * Update linear terms from Riccati backward pass
     */
    void backward_pass_grad(TinySolver *solver)
    {
        for (int i = NHORIZON - 2; i >= 0; i--)
        {
            (solver->work->d.col(i)).noalias() = solver->cache->Quu_inv * (solver->work->Bdyn.transpose() * solver->work->p.col(i + 1) + solver->work->r.col(i) + solver->cache->BPf);
            (solver->work->p.col(i)).noalias() = solver->work->q.col(i) + solver->cache->AmBKt.lazyProduct(solver->work->p.col(i + 1)) - (solver->cache->Kinf.transpose()).lazyProduct(solver->work->r.col(i)) + solver->cache->APf;
        }
    }

    /**
     * Use LQR feedback policy to roll out trajectory
     */
    void forward_pass(TinySolver *solver)
    {
        for (int i = 0; i < NHORIZON - 1; i++)
        {
            // std::cout << "Kinf: " << solver->cache->Kinf << std::endl;
            // std::cout << "d: " << solver->work->d.col(i) << std::endl;
            (solver->work->u.col(i)).noalias() = -solver->cache->Kinf.lazyProduct(solver->work->x.col(i)) - solver->work->d.col(i);
            (solver->work->x.col(i + 1)).noalias() = solver->work->Adyn.lazyProduct(solver->work->x.col(i)) + solver->work->Bdyn.lazyProduct(solver->work->u.col(i)) + solver->work->fdyn;
        }
    }

    /**
     * Project a vector s onto the second order cone defined by mu
     * @param s, mu
     * @return projection onto cone if s is outside of cone. Return s if s is inside cone
    */
    Matrix<tinytype, 3, 1> project_soc(Matrix<tinytype, 3, 1> s, float mu) {
        tinytype u0 = s(Eigen::placeholders::last) * mu;
        Matrix<tinytype, 2, 1> u1 = s.head(2);
        float a = u1.norm();
        Matrix<tinytype, 3, 1> cone_origin;
        cone_origin.setZero();
        // std::cout << s << std::endl;
        // std::cout << u0 << std::endl;
        // std::cout << u1 << std::endl;
        // std::cout << a << std::endl;

        if (a <= -u0) { // below cone
            return cone_origin;
        }
        else if (a <= u0) { // in cone
            return s;
        }
        else if (a >= abs(u0)) { // outside cone
            Matrix<tinytype, 3, 1> u2(u1.size() + 1);
            u2 << u1, a/mu;
            return 0.5 * (1 + u0/a) * u2;
        }
        else {
            return cone_origin;
        }
    }

    /**
     * Project slack (auxiliary) variables into their feasible domain, defined by
     * projection functions related to each constraint
     */
    void update_slack(TinySolver *solver)
    {
        /* Update slack variables */

        // Update box slack variables for state
        solver->work->bounds->vnew = solver->work->x + solver->work->bounds->g;
        
        // Update box slack variables for input
        solver->work->bounds->znew = solver->work->u + solver->work->bounds->y;

        // Update second order cone slack variables for state
        if (solver->settings->en_state_soc && NUM_STATE_CONES > 0) {
            for (int i=0; i<NUM_STATE_CONES; i++) {
                solver->work->socs->vcnew[i] = solver->work->x + solver->work->socs->gc[i];
            }
        }

        // Update second order cone slack variables for input
        if (solver->settings->en_input_soc && NUM_INPUT_CONES > 0) {
            for (int i=0; i<NUM_INPUT_CONES; i++) {
                solver->work->socs->zcnew[i] = solver->work->u + solver->work->socs->yc[i];
            }
        }

        /* Project slack variables. Only box and cone constraints are supported */

        // Project box constraints on state
        if (solver->settings->en_state_bound) {
            solver->work->bounds->vnew = solver->work->bounds->x_max.cwiseMin(solver->work->bounds->x_min.cwiseMax(solver->work->bounds->vnew));
        }
        
        // Project box constraints on input
        if (solver->settings->en_input_bound) {
            solver->work->bounds->znew = solver->work->bounds->u_max.cwiseMin(solver->work->bounds->u_min.cwiseMax(solver->work->bounds->znew));
        }

        // Project second order cone constraints on state
        for (int k=0; k<NHORIZON; k++) {
            if (solver->settings->en_state_soc && NUM_STATE_CONES > 0) {
                for (int i=0; i<NUM_STATE_CONES; i++) {
                    int start = solver->work->socs->Acx[i];
                    int num_xs = solver->work->socs->qcx[i];
                    // Eigen Block-API: A_matrix.block(start_row, start_col, num_rows, num_cols)
                    solver->work->socs->vcnew[i].block(start, k, num_xs, 1) = project_soc(solver->work->socs->vcnew[i].block(start, k, num_xs, 1), solver->work->socs->cx[i]);
                }
            }
        }

        // Project second order cone constraints on input
        for (int k=0; k<NHORIZON-1; k++) {
            if (solver->settings->en_input_soc && NUM_INPUT_CONES > 0) {
                for (int i=0; i<NUM_INPUT_CONES; i++) {
                    int start = solver->work->socs->Acu[i];
                    int num_us = solver->work->socs->qcu[i];
                    // Eigen Block-API: A_matrix.block(start_row, start_col, num_rows, num_cols)
                    solver->work->socs->zcnew[i].block(start, k, num_us, 1) = project_soc(solver->work->socs->zcnew[i].block(start, k, num_us, 1), solver->work->socs->cu[i]);
                }
            }
        }
    }

    /**
     * Update next iteration of dual variables by performing the augmented
     * lagrangian multiplier update
     */
    void update_dual(TinySolver *solver)
    {
        // Update box slack variables for state
        solver->work->bounds->g = solver->work->bounds->g + solver->work->x - solver->work->bounds->vnew;
        
        // Update box slack variables for input
        solver->work->bounds->y = solver->work->bounds->y + solver->work->u - solver->work->bounds->znew;

        // Update second order cone slack variables for state
        if (solver->settings->en_state_soc && NUM_STATE_CONES > 0) {
            for (int i=0; i<NUM_STATE_CONES; i++) {
                solver->work->socs->gc[i] = solver->work->socs->gc[i] + solver->work->x - solver->work->socs->vcnew[i];
            }
        }

        // Update second order cone slack variables for input
        if (solver->settings->en_input_soc && NUM_INPUT_CONES > 0) {
            for (int i=0; i<NUM_INPUT_CONES; i++) {
                solver->work->socs->yc[i] = solver->work->socs->yc[i] + solver->work->u - solver->work->socs->zcnew[i];
            }
        }
    }

    /**
     * Update linear control cost terms in the Riccati feedback using the changing
     * slack and dual variables from ADMM
     */
    void update_linear_cost(TinySolver *solver)
    {
        solver->work->r = -(solver->work->Uref.array().colwise() * solver->work->R.array()); // Uref = 0 so commented out for speed up. Need to uncomment if using Uref
        (solver->work->r).noalias() += -solver->cache->rho * (solver->work->bounds->znew - solver->work->bounds->y);
        if (solver->settings->en_input_soc && NUM_INPUT_CONES > 0) {
            for (int i=0; i<NUM_INPUT_CONES; i++) {
                (solver->work->r).noalias() -= solver->cache->rho * (solver->work->socs->zcnew[i] - solver->work->socs->yc[i]);
            }
        }
        solver->work->q = -(solver->work->Xref.array().colwise() * solver->work->Q.array());
        (solver->work->q).noalias() -= solver->cache->rho * (solver->work->bounds->vnew - solver->work->bounds->g);
        if (solver->settings->en_state_soc && NUM_STATE_CONES > 0) {
            for (int i=0; i<NUM_STATE_CONES; i++) {
                (solver->work->q).noalias() -= solver->cache->rho * (solver->work->socs->vcnew[i] - solver->work->socs->gc[i]);
            }
        }
        solver->work->p.col(NHORIZON - 1) = -(solver->work->Xref.col(NHORIZON - 1).transpose().lazyProduct(solver->cache->Pinf));
        solver->work->p.col(NHORIZON - 1) -= solver->cache->rho * (solver->work->bounds->vnew.col(NHORIZON - 1) - solver->work->bounds->g.col(NHORIZON - 1));
        if (solver->settings->en_state_soc && NUM_STATE_CONES > 0) {
            for (int i=0; i<NUM_STATE_CONES; i++) {
                solver->work->p.col(NHORIZON - 1) -= solver->cache->rho * (solver->work->socs->vcnew[i].col(NHORIZON - 1) - solver->work->socs->gc[i].col(NHORIZON - 1));
            }
        }
    }

    static tinytype max(tinytype a, tinytype b) {
        return a > b ? a : b;
    }

    /**
     * Check for termination condition by evaluating whether the largest absolute
     * primal and dual residuals for states and inputs are below threhold.
     */
    bool termination_condition(TinySolver *solver)
    {
        if (solver->work->iter % solver->settings->check_termination == 0)
        {
            solver->work->primal_residual_state = (solver->work->x - solver->work->bounds->vnew).cwiseAbs().maxCoeff();
            solver->work->dual_residual_state = ((solver->work->bounds->v - solver->work->bounds->vnew).cwiseAbs().maxCoeff()) * solver->cache->rho;
            if (solver->settings->en_state_soc && NUM_STATE_CONES > 0) {
                for (int i=0; i<NUM_STATE_CONES; i++) {
                    solver->work->primal_residual_state = max(solver->work->primal_residual_state, (solver->work->x - solver->work->socs->vcnew[i]).cwiseAbs().maxCoeff());
                    solver->work->dual_residual_state = max(solver->work->dual_residual_state, (solver->work->socs->vc[i] - solver->work->socs->vcnew[i]).cwiseAbs().maxCoeff() * solver->cache->rho);
                }
            }

            solver->work->primal_residual_input = (solver->work->u - solver->work->bounds->znew).cwiseAbs().maxCoeff();
            solver->work->dual_residual_input = ((solver->work->bounds->z - solver->work->bounds->znew).cwiseAbs().maxCoeff()) * solver->cache->rho;
            if (solver->settings->en_input_soc && NUM_INPUT_CONES > 0) {
                for (int i=0; i<NUM_INPUT_CONES; i++) {
                    solver->work->primal_residual_input = max(solver->work->primal_residual_input, (solver->work->u - solver->work->socs->zcnew[i]).cwiseAbs().maxCoeff());
                    solver->work->dual_residual_input = max(solver->work->dual_residual_input, (solver->work->socs->zc[i] - solver->work->socs->zcnew[i]).cwiseAbs().maxCoeff() * solver->cache->rho);
                }
            }

            if (solver->work->primal_residual_state < solver->settings->abs_pri_tol &&
                solver->work->primal_residual_input < solver->settings->abs_pri_tol &&
                solver->work->dual_residual_state < solver->settings->abs_dua_tol &&
                solver->work->dual_residual_input < solver->settings->abs_dua_tol)
            {
                return true;                 
            }
        }
        return false;
    }

    int tiny_solve(TinySolver *solver)
    {
        // Initialize variables
        solver->work->status = 11; // TINY_UNSOLVED

        for (int i = 0; i < solver->settings->max_iter; i++)
        {
            // Solve linear system with Riccati and roll out to get new trajectory
            backward_pass_grad(solver);
            forward_pass(solver);

            // Project slack variables into feasible domain
            update_slack(solver);

            // Compute next iteration of dual variables
            update_dual(solver);

            // Update linear control cost terms using reference trajectory, duals, and slack variables
            update_linear_cost(solver);
            
            solver->work->iter = i + 1;

            // Check for whether cost is ~minimized~ by calculating residuals
            // if (termination_condition(solver)) {
            //     solver->work->status = 1; // TINY_SOLVED
            //     return 0;
            // }

            // Save previous slack variables
            solver->work->bounds->v = solver->work->bounds->vnew;
            solver->work->bounds->z = solver->work->bounds->znew;
            if (solver->settings->en_state_soc && NUM_STATE_CONES > 0) {
                for (int i=0; i<NUM_STATE_CONES; i++) {
                    solver->work->socs->vc[i] = solver->work->socs->vcnew[i];
                }
            }
            if (solver->settings->en_input_soc && NUM_INPUT_CONES > 0) {
                for (int i=0; i<NUM_INPUT_CONES; i++) {
                    solver->work->socs->zc[i] = solver->work->socs->zcnew[i];
                }
            }
        }
        return 1;
    }

} /* extern "C" */