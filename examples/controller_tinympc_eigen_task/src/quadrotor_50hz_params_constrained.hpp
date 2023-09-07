#pragma once

#include <tinympc/types.hpp>

tinytype rho_constrained_value = 63.0;

tinytype Adyn_constrained_data[NSTATES*NSTATES] = {
  1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0039240,	-0.0000000,	0.0200000,	0.0000000,	0.0000000,	0.0000000,	0.0000131,	-0.0000000,	
  0.0000000,	1.0000000,	0.0000000,	-0.0039240,	0.0000000,	-0.0000000,	0.0000000,	0.0200000,	0.0000000,	-0.0000131,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0200000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0100000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.0000000,	1.0000000,	-0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0100000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0000000,	0.0100000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.3924000,	-0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0019620,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	-0.3924000,	0.0000000,	-0.0000000,	0.0000000,	1.0000000,	0.0000000,	-0.0019620,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	1.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	-0.0000000,	0.0000000,	1.0000000	
};

tinytype Bdyn_constrained_data[NSTATES*NINPUTS] = {
  -0.0000181,	0.0000199,	0.0000182,	-0.0000200,	
  0.0000180,	0.0000198,	-0.0000180,	-0.0000198,	
  0.0008409,	0.0008409,	0.0008409,	0.0008409,	
  -0.0275355,	-0.0303234,	0.0275663,	0.0302926,	
  -0.0276707,	0.0304278,	0.0277570,	-0.0305141,	
  0.0019748,	-0.0007224,	-0.0027844,	0.0015320,	
  -0.0036193,	0.0039800,	0.0036306,	-0.0039912,	
  0.0036016,	0.0039663,	-0.0036057,	-0.0039623,	
  0.0840857,	0.0840857,	0.0840857,	0.0840857,	
  -5.5070921,	-6.0646807,	5.5132527,	6.0585201,	
  -5.5341404,	6.0855684,	5.5513900,	-6.1028180,	
  0.3949542,	-0.1444728,	-0.5568752,	0.3063938	
};

tinytype Kinf_constrained_data[NINPUTS*NSTATES] = {
  -0.0787031,	0.0714633,	3.4605149,	-0.5794468,	-0.6719511,	-0.7884844,	-0.0887439,	0.0793523,	0.7009118,	-0.0564452,	-0.0702396,	-0.3730859,	
  0.0729986,	0.0555216,	3.4605149,	-0.4072580,	0.6272625,	0.7898947,	0.0824700,	0.0599332,	0.7009118,	-0.0335947,	0.0661325,	0.3735096,	
  0.0531107,	-0.0612374,	3.4605149,	0.4520162,	0.3531661,	-0.7928835,	0.0558797,	-0.0662187,	0.7009118,	0.0377071,	0.0233764,	-0.3743704,	
  -0.0474062,	-0.0657474,	3.4605149,	0.5346886,	-0.3084775,	0.7914733,	-0.0496057,	-0.0730668,	0.7009118,	0.0523328,	-0.0192693,	0.3739466	
};

tinytype Pinf_constrained_data[NSTATES*NSTATES] = {
  8941.5746753,	-1.2645578,	0.0000000,	10.1787667,	10947.9250784,	70.0427674,	3140.7061216,	-1.4169267,	0.0000000,	0.8575601,	56.0952691,	26.5040484,	
  -1.2645578,	8939.5221005,	0.0000000,	-10931.0807924,	-10.1794688,	-28.0078213,	-1.4169621,	3138.3859548,	-0.0000000,	-54.6851716,	-0.8576504,	-10.5981920,	
  0.0000000,	0.0000000,	101910.7869841,	-0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	7626.6796724,	-0.0000000,	-0.0000000,	-0.0000000,	
  10.1787667,	-10931.0807924,	-0.0000000,	80266.5665256,	97.1534113,	328.8433266,	12.0405740,	-11767.4918491,	0.0000000,	410.4903305,	9.7477987,	131.5641090,	
  10947.9250784,	-10.1794688,	0.0000000,	97.1534113,	80435.9291815,	822.2691049,	11787.8249963,	-12.0410714,	0.0000000,	9.7475769,	427.9222600,	328.9688813,	
  70.0427674,	-28.0078213,	-0.0000000,	328.8433266,	822.2691049,	21880.6257006,	88.6432573,	-35.4474550,	-0.0000000,	41.0123832,	102.5436157,	1710.9850564,	
  3140.7061216,	-1.4169621,	0.0000000,	12.0405740,	11787.8249963,	88.6432573,	2890.1126261,	-1.6157855,	0.0000000,	1.0646783,	61.1105344,	33.9916431,	
  -1.4169267,	3138.3859548,	0.0000000,	-11767.4918491,	-12.0410714,	-35.4474550,	-1.6157855,	2887.4331242,	-0.0000000,	-59.3034514,	-1.0647534,	-13.5930305,	
  -0.0000000,	0.0000000,	7626.6796724,	-0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	1501.9831337,	-0.0000000,	-0.0000000,	-0.0000000,	
  0.8575601,	-54.6851716,	-0.0000000,	410.4903305,	9.7475769,	41.0123832,	1.0646783,	-59.3034514,	-0.0000000,	68.7971161,	1.3373385,	19.2716309,	
  56.0952691,	-0.8576504,	0.0000000,	9.7477987,	427.9222600,	102.5436157,	61.1105344,	-1.0647534,	-0.0000000,	1.3373385,	71.3285642,	48.1833928,	
  26.5040484,	-10.5981920,	-0.0000000,	131.5641090,	328.9688813,	1710.9850564,	33.9916431,	-13.5930305,	-0.0000000,	19.2716309,	48.1833928,	833.6608417	
};

tinytype Quu_inv_constrained_data[NINPUTS*NINPUTS] = {
  0.0026021,	-0.0001661,	0.0024878,	-0.0001638,	
  -0.0001661,	0.0025892,	-0.0001597,	0.0024967,	
  0.0024878,	-0.0001597,	0.0025881,	-0.0001561,	
  -0.0001638,	0.0024967,	-0.0001561,	0.0025833	
};

tinytype AmBKt_constrained_data[NSTATES*NSTATES] = {
  0.9999952,	-0.0000000,	-0.0000000,	0.0000184,	-0.0073197,	0.0004287,	-0.0009574,	-0.0000024,	-0.0000000,	0.0036871,	-1.4639418,	0.0857314,	
  -0.0000000,	0.9999952,	0.0000000,	0.0073311,	-0.0000184,	-0.0001708,	-0.0000024,	-0.0009589,	0.0000000,	1.4662249,	-0.0036842,	-0.0341604,	
  0.0000000,	0.0000000,	0.9883608,	-0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	-1.1639194,	-0.0000000,	0.0000000,	0.0000000,	
  0.0000001,	-0.0038867,	-0.0000000,	0.9430377,	0.0001272,	0.0012895,	0.0000166,	-0.3849493,	-0.0000000,	-11.3924575,	0.0254451,	0.2579086,	
  0.0038868,	-0.0000001,	0.0000000,	0.0001274,	0.9431047,	0.0032360,	0.3849581,	-0.0000167,	0.0000000,	0.0254732,	-11.3790647,	0.6471973,	
  0.0000002,	-0.0000001,	0.0000000,	0.0001221,	0.0003064,	0.9987075,	0.0000401,	-0.0000160,	0.0000000,	0.0244131,	0.0612815,	-0.2585061,	
  0.0199947,	-0.0000000,	0.0000000,	0.0000195,	-0.0080297,	0.0004664,	0.9989497,	-0.0000025,	0.0000000,	0.0038915,	-1.6059422,	0.0932813,	
  -0.0000000,	0.0199947,	0.0000000,	0.0080412,	-0.0000194,	-0.0001859,	-0.0000025,	0.9989482,	0.0000000,	1.6082337,	-0.0038881,	-0.0371702,	
  -0.0000000,	0.0000000,	0.0176425,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.7642533,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	-0.0000097,	-0.0000000,	0.0048023,	0.0000106,	0.0001120,	0.0000014,	-0.0012821,	-0.0000000,	-0.0395382,	0.0021177,	0.0224035,	
  0.0000097,	-0.0000000,	-0.0000000,	0.0000106,	0.0048073,	0.0002811,	0.0012828,	-0.0000014,	-0.0000000,	0.0021204,	-0.0385388,	0.0562175,	
  0.0000001,	-0.0000000,	0.0000000,	0.0000452,	0.0001134,	0.0093913,	0.0000148,	-0.0000059,	0.0000000,	0.0090337,	0.0226762,	0.8782613	
};

tinytype coeff_d2p_constrained_data[NSTATES*NINPUTS] = {
  0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  -0.0000000,	-0.0000000,	0.0000000,	0.0000000,	
  -0.0000000,	-0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	0.0000000,	-0.0000000,	-0.0000000,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	
  0.0000000,	-0.0000000,	-0.0000000,	0.0000000,	
  -0.0000000,	-0.0000000,	0.0000000,	0.0000000,	
  -0.0000000,	-0.0000000,	0.0000000,	-0.0000000,	
  0.0000000,	0.0000000,	-0.0000000,	-0.0000000,	
  0.0000000,	-0.0000000,	0.0000000,	0.0000000,	
  0.0000000,	-0.0000000,	0.0000000,	-0.0000000	
};

tinytype Q_constrained_data[NSTATES]= {100.0000000,	100.0000000,	10000.0000000,	4.0000000,	4.0000000,	400.0000000,	4.0000000,	4.0000000,	4.0000000,	2.0408163,	2.0408163,	4.0000000};

tinytype Qf_constrained_data[NSTATES]= {100.0000000,	100.0000000,	10000.0000000,	4.0000000,	4.0000000,	400.0000000,	4.0000000,	4.0000000,	4.0000000,	2.0408163,	2.0408163,	4.0000000};

tinytype R_constrained_data[NINPUTS]= {100.0000000,	100.0000000,	100.0000000,	100.0000000};

