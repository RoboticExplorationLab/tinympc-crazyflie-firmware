#pragma once

#include <tinympc/types.hpp>

static const tinytype rho_value = 1.0;

static const tinytype Adyn_data[NSTATES*NSTATES]  = {
	1.000000f, 0.000000f, 0.000000f, 0.020000f, 0.000000f, 0.000000f, 
	0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.020000f, 0.000000f, 
	0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 0.020000f, 
	0.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 0.000000f, 1.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 0.000000f, 0.000000f, 1.000000f
};

static const tinytype Bdyn_data[NSTATES*NINPUTS]  = {
	0.000200f, 0.000000f, 0.000000f, 
	0.000000f, 0.000200f, 0.000000f, 
	0.000000f, 0.000000f, 0.000200f, 
	0.020000f, 0.000000f, 0.000000f, 
	0.000000f, 0.020000f, 0.000000f, 
	0.000000f, 0.000000f, 0.020000f
};

static const tinytype fdyn_data[NSTATES]  = {0.000000f, 0.000000f, -0.001962f, 0.000000f, 0.000000f, -0.196200f};

static const tinytype Q_data[NSTATES] = {101.000000f, 101.000000f, 101.000000f, 101.000000f, 101.000000f, 101.000000f};

static const tinytype R_data[NINPUTS] = {2.000000f, 2.000000f, 2.000000f};

static const tinytype Kinf_data[NINPUTS*NSTATES]  = {
	6.557462f, 0.000000f, 0.000000f, 7.491010f, 0.000000f, 0.000000f, 
	0.000000f, 6.557462f, 0.000000f, 0.000000f, 7.491010f, 0.000000f, 
	0.000000f, 0.000000f, 6.557462f, 0.000000f, 0.000000f, 7.491010f
};

static const tinytype Pinf_data[NSTATES*NSTATES]  = {
	5768.939355f, 0.000000f, 0.000000f, 712.425607f, 0.000000f, 0.000000f, 
	0.000000f, 5768.939355f, 0.000000f, 0.000000f, 712.425607f, 0.000000f, 
	0.000000f, 0.000000f, 5768.939355f, 0.000000f, 0.000000f, 712.425607f, 
	712.425607f, 0.000000f, 0.000000f, 857.225279f, 0.000000f, 0.000000f, 
	0.000000f, 712.425607f, 0.000000f, 0.000000f, 857.225279f, 0.000000f, 
	0.000000f, 0.000000f, 712.425607f, 0.000000f, 0.000000f, 857.225279f
};

static const tinytype Quu_inv_data[NINPUTS*NINPUTS]  = {
	0.425746f, 0.000000f, 0.000000f, 
	0.000000f, 0.425746f, 0.000000f, 
	0.000000f, 0.000000f, 0.425746f
};

static const tinytype AmBKt_data[NSTATES*NSTATES]  = {
	0.998689f, 0.000000f, 0.000000f, -0.131149f, 0.000000f, 0.000000f, 
	0.000000f, 0.998689f, 0.000000f, 0.000000f, -0.131149f, 0.000000f, 
	0.000000f, 0.000000f, 0.998689f, 0.000000f, 0.000000f, -0.131149f, 
	0.018502f, 0.000000f, 0.000000f, 0.850180f, 0.000000f, 0.000000f, 
	0.000000f, 0.018502f, 0.000000f, 0.000000f, 0.850180f, 0.000000f, 
	0.000000f, 0.000000f, 0.018502f, 0.000000f, 0.000000f, 0.850180f
};

static const tinytype APf_data[NSTATES]  = {0.000000f, 0.000000f, -128.657407f, 0.000000f, 0.000000f, -146.973621f};

static const tinytype BPf_data[NINPUTS]  = {0.000000f, 0.000000f, -3.421927f};

