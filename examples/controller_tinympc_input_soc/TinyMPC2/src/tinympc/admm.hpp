#pragma once

#include "types.hpp"

#ifdef __cplusplus
extern "C"
{
#endif

    int tiny_solve(TinySolver *solver);
    
    // Helper functions
    Matrix<tinytype, 3, 1> project_soc(Matrix<tinytype, 3, 1> s, float mu);
    bool termination_condition(TinySolver *solver);
    void reset_dual(TinySolver *solver);
    void reset_problem(TinySolver *solver);

    // Core ADMM functions
    void backward_pass_grad(TinySolver *solver);
    void forward_pass(TinySolver *solver);
    void update_slack(TinySolver *solver);
    void update_dual(TinySolver *solver);
    void update_linear_cost(TinySolver *solver);

#ifdef __cplusplus
}
#endif