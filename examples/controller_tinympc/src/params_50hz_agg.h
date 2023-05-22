static sfloat Kinf_data[NINPUTS*NSTATES] = {
  -0.128601f,0.124073f,0.104076f,-0.099548f,
  0.123295f,0.108751f,-0.113533f,-0.118513f,
  0.561104f,0.561104f,0.561104f,0.561104f,
  -0.582210f,-0.453563f,0.481187f,0.554586f,
  -0.653431f,0.626663f,0.400368f,-0.373600f,
  -0.718483f,0.734378f,-0.771968f,0.756074f,
  -0.111727f,0.107523f,0.084404f,-0.080201f,
  0.104973f,0.089684f,-0.094084f,-0.100573f,
  0.247898f,0.247898f,0.247898f,0.247898f,
  -0.044768f,-0.027011f,0.029233f,0.042546f,
  -0.056286f,0.054110f,0.017418f,-0.015241f,
  -0.304774f,0.309013f,-0.319021f,0.314782f,
};
static sfloat Pinf_data[NSTATES*NSTATES] = {
  4209.646851f,-9.960037f,0.000000f,44.607452f,2962.773612f,248.065311f,1105.581252f,-8.488692f,0.000000f,2.571030f,27.431725f,72.233291f,
  -9.960037f,4194.759153f,-0.000000f,-2894.719445f,-44.705433f,-98.807118f,-8.494953f,1092.751931f,-0.000000f,-23.640244f,-2.580971f,-28.778217f,
  0.000000f,0.000000f,13806.362214f,-0.000000f,0.000000f,-0.000000f,0.000000f,0.000000f,2755.536630f,-0.000000f,-0.000000f,-0.000000f,
  44.607452f,-2894.719445f,-0.000000f,11667.072105f,270.544634f,904.823642f,41.734885f,-2350.452736f,-0.000000f,105.003259f,22.196284f,294.780602f,
  2962.773612f,-44.705433f,0.000000f,270.544634f,12121.220336f,2266.351140f,2416.555110f,-41.789461f,0.000000f,22.180288f,144.167080f,738.127706f,
  248.065311f,-98.807118f,-0.000000f,904.823642f,2266.351140f,46450.417664f,267.031512f,-106.469291f,-0.000000f,108.282671f,270.968141f,4480.714723f,
  1105.581252f,-8.494953f,0.000000f,41.734885f,2416.555110f,267.031512f,784.331061f,-7.447321f,0.000000f,2.668711f,23.983719f,80.157564f,
  -8.488692f,1092.751931f,-0.000000f,-2350.452736f,-41.789461f,-106.469291f,-7.447321f,772.859713f,-0.000000f,-19.742945f,-2.675270f,-31.968747f,
  0.000000f,0.000000f,2755.536630f,-0.000000f,0.000000f,-0.000000f,0.000000f,0.000000f,1192.975645f,-0.000000f,-0.000000f,-0.000000f,
  2.571030f,-23.640244f,0.000000f,105.003259f,22.180288f,108.282671f,2.668711f,-19.742945f,-0.000000f,10.710048f,3.064171f,45.015925f,
  27.431725f,-2.580971f,-0.000000f,22.196284f,144.167080f,270.968141f,23.983719f,-2.675270f,-0.000000f,3.064171f,16.618828f,112.610359f,
  72.233291f,-28.778217f,-0.000000f,294.780602f,738.127706f,4480.714723f,80.157564f,-31.968747f,-0.000000f,45.015925f,112.610359f,1902.857318f,
};
static sfloat Quu_inv_data[NINPUTS*NINPUTS] = {
  0.001286f,0.000003f,0.000729f,-0.000003f,
  0.000003f,0.001261f,-0.000001f,0.000751f,
  0.000729f,-0.000001f,0.001277f,0.000009f,
  -0.000003f,0.000751f,0.000009f,0.001258f,
};
static sfloat AmBKt_data[NSTATES*NSTATES] = {
  0.999991f,-0.000000f,0.000000f,0.000001f,0.004193f,0.000002f,0.019992f,-0.000000f,-0.000000f,0.000000f,0.000011f,0.000000f,
  -0.000000f,0.999991f,-0.000000f,-0.004193f,-0.000001f,-0.000001f,-0.000000f,0.019992f,0.000000f,-0.000011f,-0.000000f,-0.000000f,
  0.000000f,0.000000f,0.997840f,-0.000000f,-0.000000f,-0.000000f,0.000000f,-0.000000f,0.019046f,0.000000f,-0.000000f,-0.000000f,
  0.000361f,0.013158f,0.000000f,0.941288f,0.001266f,0.000846f,0.000281f,0.011036f,-0.000000f,0.005932f,0.000071f,0.000232f,
  -0.013008f,-0.000355f,0.000000f,0.001233f,0.941472f,0.002229f,-0.010941f,-0.000276f,-0.000000f,0.000069f,0.005926f,0.000613f,
  0.000771f,-0.000294f,0.000000f,0.001287f,0.003365f,0.998667f,0.000644f,-0.000246f,0.000000f,0.000083f,0.000218f,0.009465f,
  -0.001432f,-0.000039f,-0.000000f,0.000136f,0.385956f,0.000245f,0.998795f,-0.000030f,-0.000000f,0.000008f,0.001358f,0.000067f,
  -0.000040f,-0.001449f,-0.000000f,-0.385935f,-0.000139f,-0.000093f,-0.000031f,0.998785f,0.000000f,-0.001359f,-0.000008f,-0.000026f,
  0.000000f,0.000000f,-0.215994f,-0.000000f,-0.000000f,-0.000000f,0.000000f,-0.000000f,0.904573f,0.000000f,-0.000000f,-0.000000f,
  0.072170f,2.631523f,0.000000f,-11.742440f,0.253236f,0.169103f,0.056275f,2.207279f,-0.000000f,0.186471f,0.014218f,0.046483f,
  -2.601654f,-0.071096f,-0.000000f,0.246657f,-11.705669f,0.445747f,-2.188279f,-0.055257f,-0.000000f,0.013726f,0.185287f,0.122511f,
  0.154189f,-0.058759f,-0.000000f,0.257472f,0.673000f,-0.266518f,0.128743f,-0.049129f,0.000000f,0.016699f,0.043573f,0.892986f,
};
static sfloat coeff_d2p_data[NSTATES*NINPUTS] = {
  0.000018f,-0.000017f,-0.000001f,0.000020f,0.000022f,0.000055f,0.000006f,-0.000006f,-0.000000f,0.000000f,0.000001f,0.000010f,
  -0.000018f,-0.000017f,-0.000001f,0.000017f,-0.000023f,-0.000059f,-0.000006f,-0.000005f,-0.000000f,-0.000000f,-0.000001f,-0.000010f,
  -0.000016f,0.000016f,-0.000001f,-0.000017f,-0.000016f,0.000071f,-0.000005f,0.000005f,-0.000000f,0.000000f,0.000000f,0.000010f,
  0.000017f,0.000017f,-0.000001f,-0.000020f,0.000016f,-0.000067f,0.000005f,0.000006f,-0.000000f,-0.000000f,-0.000000f,-0.000010f,
};
static sfloat Q_data[NSTATES*NSTATES] = {
  100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,625.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1111.111111f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,25.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,25.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,6.250000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,6.250000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,6.250000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,100.000000f,
};
static sfloat R_data[NINPUTS*NINPUTS] = {
  400.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,400.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,400.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,400.000000f,
};