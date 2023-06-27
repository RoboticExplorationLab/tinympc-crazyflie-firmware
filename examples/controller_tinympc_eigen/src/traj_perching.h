static const float X_ref_data[256][12] = {
{-0.0000,0.0000,1.0000,-0.0000,0.0000,0.0000,0.0000,0.0000,-0.0000,-0.0000,0.0000,-0.0000},
{0.0000,0.0000,1.0000,-0.0000,0.0037,0.0000,0.0012,0.0000,-0.0000,-0.0000,0.2962,0.0000},
{0.0000,0.0000,1.0000,-0.0000,0.0074,0.0000,0.0024,0.0000,-0.0001,-0.0000,0.5924,0.0000},
{0.0000,0.0000,1.0000,-0.0000,0.0111,0.0000,0.0036,0.0000,-0.0001,-0.0000,0.8886,0.0000},
{0.0001,0.0000,1.0000,-0.0000,0.0148,0.0000,0.0048,0.0000,-0.0002,-0.0000,1.1848,0.0000},
{0.0001,0.0000,1.0000,-0.0000,0.0185,0.0000,0.0061,0.0000,-0.0002,-0.0000,1.4810,0.0000},
{0.0001,0.0000,1.0000,-0.0000,0.0222,0.0000,0.0073,0.0000,-0.0003,-0.0000,1.7772,0.0000},
{0.0001,0.0000,1.0000,-0.0000,0.0259,0.0000,0.0085,0.0000,-0.0003,-0.0000,2.0734,0.0000},
{0.0001,0.0000,1.0000,-0.0000,0.0296,0.0000,0.0097,0.0000,-0.0004,-0.0000,2.3696,0.0000},
{0.0001,0.0000,1.0000,-0.0000,0.0333,0.0000,0.0109,0.0000,-0.0004,-0.0000,2.6658,0.0000},
{0.0002,0.0000,1.0000,-0.0000,0.0370,0.0000,0.0121,0.0000,-0.0005,-0.0000,2.9620,0.0000},
{0.0002,0.0000,1.0000,-0.0000,0.0407,0.0000,0.0133,0.0000,-0.0005,-0.0000,3.2582,0.0000},
{0.0002,0.0000,1.0000,-0.0000,0.0444,0.0000,0.0145,0.0000,-0.0006,-0.0000,3.5544,0.0000},
{0.0002,0.0000,1.0000,-0.0000,0.0481,0.0000,0.0157,0.0000,-0.0006,-0.0000,3.8506,0.0000},
{0.0002,0.0000,1.0000,-0.0000,0.0518,0.0000,0.0169,0.0000,-0.0007,-0.0000,4.1468,0.0000},
{0.0002,0.0000,1.0000,-0.0000,0.0555,0.0000,0.0182,0.0000,-0.0007,-0.0000,4.4430,0.0000},
{0.0004,0.0000,1.0000,-0.0000,0.0617,0.0000,0.0250,0.0000,-0.0008,-0.0000,4.3398,0.0000},
{0.0006,0.0000,1.0000,-0.0000,0.0678,0.0000,0.0318,0.0000,-0.0009,-0.0000,4.2367,0.0000},
{0.0008,0.0000,1.0000,-0.0000,0.0739,0.0000,0.0387,0.0000,-0.0009,-0.0000,4.1335,0.0000},
{0.0011,0.0000,1.0000,-0.0000,0.0800,0.0000,0.0455,0.0000,-0.0010,-0.0000,4.0303,0.0000},
{0.0013,0.0000,1.0000,-0.0000,0.0861,0.0000,0.0524,0.0000,-0.0011,-0.0000,3.9272,0.0000},
{0.0015,0.0000,1.0000,-0.0000,0.0922,0.0000,0.0592,0.0000,-0.0012,-0.0000,3.8240,0.0000},
{0.0017,0.0000,1.0000,-0.0000,0.0983,0.0000,0.0661,0.0000,-0.0013,-0.0000,3.7209,0.0000},
{0.0019,0.0000,0.9999,-0.0000,0.1045,0.0000,0.0729,0.0000,-0.0013,-0.0000,3.6177,0.0000},
{0.0021,0.0000,0.9999,-0.0000,0.1106,0.0000,0.0797,0.0000,-0.0014,-0.0000,3.5146,0.0000},
{0.0023,0.0000,0.9999,-0.0000,0.1167,0.0000,0.0866,0.0000,-0.0015,-0.0000,3.4114,0.0000},
{0.0025,0.0000,0.9999,-0.0000,0.1228,0.0000,0.0934,0.0000,-0.0016,-0.0000,3.3083,0.0000},
{0.0027,0.0000,0.9999,-0.0000,0.1289,0.0000,0.1003,0.0000,-0.0017,-0.0000,3.2051,0.0000},
{0.0029,0.0000,0.9999,-0.0000,0.1350,0.0000,0.1071,0.0000,-0.0017,-0.0000,3.1020,0.0000},
{0.0031,0.0000,0.9999,-0.0000,0.1412,0.0000,0.1140,0.0000,-0.0018,-0.0000,2.9988,0.0000},
{0.0033,0.0000,0.9999,-0.0000,0.1473,0.0000,0.1208,0.0000,-0.0019,-0.0000,2.8956,0.0000},
{0.0040,0.0000,0.9999,-0.0000,0.1511,0.0000,0.1325,0.0000,-0.0020,-0.0000,2.8195,0.0000},
{0.0047,0.0000,0.9999,-0.0000,0.1550,0.0000,0.1442,0.0000,-0.0021,-0.0000,2.7434,0.0000},
{0.0054,0.0000,0.9999,-0.0000,0.1589,0.0000,0.1559,0.0000,-0.0022,-0.0000,2.6673,0.0000},
{0.0060,0.0000,0.9999,-0.0000,0.1628,0.0000,0.1675,0.0000,-0.0023,-0.0000,2.5911,0.0000},
{0.0067,0.0000,0.9999,-0.0000,0.1666,0.0000,0.1792,0.0000,-0.0024,-0.0000,2.5150,0.0000},
{0.0074,0.0000,0.9999,-0.0000,0.1705,0.0000,0.1909,0.0000,-0.0025,-0.0000,2.4389,0.0000},
{0.0081,0.0000,0.9999,-0.0000,0.1744,0.0000,0.2026,0.0000,-0.0026,-0.0000,2.3627,0.0000},
{0.0088,0.0000,0.9998,-0.0000,0.1783,0.0000,0.2143,0.0000,-0.0027,-0.0000,2.2866,0.0000},
{0.0094,0.0000,0.9998,-0.0000,0.1821,0.0000,0.2260,0.0000,-0.0028,-0.0000,2.2105,0.0000},
{0.0101,0.0000,0.9998,-0.0000,0.1860,0.0000,0.2377,0.0000,-0.0029,-0.0000,2.1343,0.0000},
{0.0108,0.0000,0.9998,-0.0000,0.1899,0.0000,0.2494,0.0000,-0.0030,-0.0000,2.0582,0.0000},
{0.0115,0.0000,0.9998,-0.0000,0.1938,0.0000,0.2610,0.0000,-0.0031,-0.0000,1.9821,0.0000},
{0.0122,0.0000,0.9998,-0.0000,0.1976,0.0000,0.2727,0.0000,-0.0032,-0.0000,1.9059,0.0000},
{0.0128,0.0000,0.9998,-0.0000,0.2015,0.0000,0.2844,0.0000,-0.0033,-0.0000,1.8298,0.0000},
{0.0135,0.0000,0.9998,-0.0000,0.2054,0.0000,0.2961,0.0000,-0.0034,-0.0000,1.7537,0.0000},
{0.0149,0.0000,0.9998,-0.0000,0.2076,0.0000,0.3108,0.0000,-0.0035,-0.0000,1.6993,0.0000},
{0.0162,0.0000,0.9998,-0.0000,0.2099,0.0000,0.3254,0.0000,-0.0036,-0.0000,1.6450,0.0000},
{0.0175,0.0000,0.9997,-0.0000,0.2121,0.0000,0.3400,0.0000,-0.0038,-0.0000,1.5907,0.0000},
{0.0189,0.0000,0.9997,-0.0000,0.2144,0.0000,0.3547,0.0000,-0.0039,-0.0000,1.5363,0.0000},
{0.0202,0.0000,0.9997,-0.0000,0.2166,0.0000,0.3693,0.0000,-0.0040,-0.0000,1.4820,0.0000},
{0.0216,0.0000,0.9997,-0.0000,0.2188,0.0000,0.3840,0.0000,-0.0041,0.0000,1.4277,0.0000},
{0.0229,0.0000,0.9997,-0.0000,0.2211,0.0000,0.3986,0.0000,-0.0042,0.0000,1.3733,0.0000},
{0.0243,0.0000,0.9997,-0.0000,0.2233,0.0000,0.4133,0.0000,-0.0043,0.0000,1.3190,0.0000},
{0.0256,0.0000,0.9997,-0.0000,0.2256,0.0000,0.4279,0.0000,-0.0044,0.0000,1.2647,0.0000},
{0.0270,0.0000,0.9996,-0.0000,0.2278,0.0000,0.4425,0.0000,-0.0046,0.0000,1.2103,0.0000},
{0.0283,0.0000,0.9996,-0.0000,0.2301,0.0000,0.4572,0.0000,-0.0047,0.0000,1.1560,0.0000},
{0.0296,0.0000,0.9996,-0.0000,0.2323,0.0000,0.4718,0.0000,-0.0048,0.0000,1.1017,0.0000},
{0.0310,0.0000,0.9996,-0.0000,0.2346,0.0000,0.4865,0.0000,-0.0049,0.0000,1.0473,0.0000},
{0.0323,0.0000,0.9996,-0.0000,0.2368,0.0000,0.5011,0.0000,-0.0050,0.0000,0.9930,0.0000},
{0.0337,0.0000,0.9996,-0.0000,0.2390,0.0000,0.5158,0.0000,-0.0051,0.0000,0.9387,0.0000},
{0.0358,0.0000,0.9996,-0.0000,0.2401,0.0000,0.5320,0.0000,-0.0052,0.0000,0.9006,0.0000},
{0.0379,0.0000,0.9995,-0.0000,0.2412,0.0000,0.5483,0.0000,-0.0053,0.0000,0.8625,0.0000},
{0.0400,0.0000,0.9995,-0.0000,0.2423,0.0000,0.5645,0.0000,-0.0055,0.0000,0.8244,0.0000},
{0.0422,0.0000,0.9995,-0.0000,0.2434,0.0000,0.5807,0.0000,-0.0056,0.0000,0.7863,0.0000},
{0.0443,0.0000,0.9995,-0.0000,0.2445,0.0000,0.5970,0.0000,-0.0057,0.0000,0.7482,0.0000},
{0.0464,0.0000,0.9995,-0.0000,0.2456,0.0000,0.6132,0.0000,-0.0058,0.0000,0.7102,0.0000},
{0.0485,0.0000,0.9994,-0.0000,0.2467,0.0000,0.6295,0.0000,-0.0059,0.0000,0.6721,0.0000},
{0.0506,0.0000,0.9994,-0.0000,0.2477,0.0000,0.6457,0.0000,-0.0060,0.0000,0.6340,0.0000},
{0.0528,0.0000,0.9994,-0.0000,0.2488,0.0000,0.6620,0.0000,-0.0061,0.0000,0.5959,0.0000},
{0.0549,0.0000,0.9994,-0.0000,0.2499,0.0000,0.6782,0.0000,-0.0063,0.0000,0.5578,0.0000},
{0.0570,0.0000,0.9994,-0.0000,0.2510,0.0000,0.6945,0.0000,-0.0064,0.0000,0.5197,0.0000},
{0.0591,0.0000,0.9993,-0.0000,0.2521,0.0000,0.7107,0.0000,-0.0065,0.0000,0.4817,0.0000},
{0.0612,0.0000,0.9993,-0.0000,0.2532,0.0000,0.7270,0.0000,-0.0066,0.0000,0.4436,0.0000},
{0.0634,0.0000,0.9993,-0.0000,0.2543,0.0000,0.7432,0.0000,-0.0067,0.0000,0.4055,0.0000},
{0.0655,0.0000,0.9993,-0.0000,0.2554,0.0000,0.7594,0.0000,-0.0068,0.0000,0.3674,0.0000},
{0.0684,0.0000,0.9992,-0.0000,0.2556,0.0000,0.7763,0.0000,-0.0069,0.0000,0.3398,0.0000},
{0.0714,0.0000,0.9992,-0.0000,0.2559,0.0000,0.7932,0.0000,-0.0070,0.0000,0.3122,0.0000},
{0.0743,0.0000,0.9992,-0.0000,0.2562,0.0000,0.8101,0.0000,-0.0071,0.0000,0.2847,0.0000},
{0.0773,0.0000,0.9992,-0.0000,0.2564,0.0000,0.8270,0.0000,-0.0072,0.0000,0.2571,0.0000},
{0.0802,0.0000,0.9991,-0.0000,0.2567,0.0000,0.8439,0.0000,-0.0074,0.0000,0.2295,0.0000},
{0.0832,0.0000,0.9991,-0.0000,0.2570,0.0000,0.8608,0.0000,-0.0075,0.0000,0.2019,0.0000},
{0.0862,0.0000,0.9991,-0.0000,0.2572,0.0000,0.8777,0.0000,-0.0076,0.0000,0.1743,0.0000},
{0.0891,0.0000,0.9991,-0.0000,0.2575,0.0000,0.8946,0.0000,-0.0077,0.0000,0.1467,0.0000},
{0.0921,0.0000,0.9990,-0.0000,0.2578,0.0000,0.9114,0.0000,-0.0078,0.0000,0.1192,0.0000},
{0.0950,0.0000,0.9990,-0.0000,0.2580,0.0000,0.9283,0.0000,-0.0079,0.0000,0.0916,0.0000},
{0.0980,0.0000,0.9990,-0.0000,0.2583,0.0000,0.9452,0.0000,-0.0080,0.0000,0.0640,0.0000},
{0.1009,0.0000,0.9990,-0.0000,0.2586,0.0000,0.9621,0.0000,-0.0081,0.0000,0.0364,0.0000},
{0.1039,0.0000,0.9989,-0.0000,0.2588,0.0000,0.9790,0.0000,-0.0082,0.0000,0.0088,0.0000},
{0.1068,0.0000,0.9989,-0.0000,0.2591,0.0000,0.9959,0.0000,-0.0083,0.0000,-0.0188,0.0000},
{0.1098,0.0000,0.9989,-0.0000,0.2594,0.0000,1.0128,0.0000,-0.0084,0.0000,-0.0463,0.0000},
{0.1136,0.0000,0.9989,-0.0000,0.2590,0.0000,1.0296,0.0000,-0.0085,0.0000,-0.0693,0.0000},
{0.1174,0.0000,0.9988,-0.0000,0.2587,0.0000,1.0464,0.0000,-0.0086,0.0000,-0.0922,0.0000},
{0.1212,0.0000,0.9988,0.0000,0.2583,0.0000,1.0633,0.0000,-0.0086,0.0000,-0.1152,0.0000},
{0.1250,0.0000,0.9988,0.0000,0.2579,0.0000,1.0801,0.0000,-0.0087,0.0000,-0.1381,0.0000},
{0.1288,0.0000,0.9987,0.0000,0.2576,0.0000,1.0969,0.0000,-0.0088,0.0000,-0.1611,0.0000},
{0.1326,0.0000,0.9987,0.0000,0.2572,0.0000,1.1138,0.0000,-0.0089,0.0000,-0.1840,0.0000},
{0.1364,0.0000,0.9987,0.0000,0.2568,0.0000,1.1306,0.0000,-0.0090,0.0000,-0.2070,0.0000},
{0.1402,0.0000,0.9987,0.0000,0.2565,0.0000,1.1474,0.0000,-0.0091,0.0000,-0.2299,0.0000},
{0.1440,0.0000,0.9986,0.0000,0.2561,0.0000,1.1643,0.0000,-0.0091,0.0000,-0.2529,0.0000},
{0.1478,0.0000,0.9986,0.0000,0.2557,0.0000,1.1811,0.0000,-0.0092,0.0000,-0.2759,0.0000},
{0.1516,0.0000,0.9986,0.0000,0.2554,0.0000,1.1979,0.0000,-0.0093,0.0000,-0.2988,0.0000},
{0.1554,0.0000,0.9985,0.0000,0.2550,0.0000,1.2148,0.0000,-0.0094,0.0000,-0.3218,0.0000},
{0.1591,0.0000,0.9985,0.0000,0.2546,0.0000,1.2316,0.0000,-0.0095,0.0000,-0.3447,0.0000},
{0.1629,0.0000,0.9985,0.0000,0.2543,0.0000,1.2484,0.0000,-0.0095,0.0000,-0.3677,0.0000},
{0.1667,0.0000,0.9984,0.0000,0.2539,0.0000,1.2652,0.0000,-0.0096,0.0000,-0.3906,0.0000},
{0.1714,0.0000,0.9984,0.0000,0.2530,0.0000,1.2814,0.0000,-0.0097,0.0000,-0.4148,0.0000},
{0.1760,0.0000,0.9984,0.0000,0.2520,0.0000,1.2976,0.0000,-0.0097,0.0000,-0.4391,0.0000},
{0.1806,0.0000,0.9983,0.0000,0.2511,0.0000,1.3138,0.0000,-0.0098,0.0000,-0.4633,0.0000},
{0.1853,0.0000,0.9983,0.0000,0.2501,0.0000,1.3300,0.0000,-0.0098,0.0000,-0.4875,0.0000},
{0.1899,0.0000,0.9983,0.0000,0.2491,0.0000,1.3462,0.0000,-0.0099,0.0000,-0.5118,0.0000},
{0.1945,0.0000,0.9982,0.0000,0.2482,0.0000,1.3624,0.0000,-0.0099,0.0000,-0.5360,0.0000},
{0.1991,0.0000,0.9982,0.0000,0.2472,0.0000,1.3786,0.0000,-0.0100,0.0000,-0.5602,0.0000},
{0.2038,0.0000,0.9982,0.0000,0.2463,0.0000,1.3947,0.0000,-0.0100,0.0000,-0.5845,0.0000},
{0.2084,0.0000,0.9981,0.0000,0.2453,0.0000,1.4109,0.0000,-0.0100,0.0000,-0.6087,0.0000},
{0.2130,0.0000,0.9981,0.0000,0.2444,0.0000,1.4271,0.0000,-0.0101,0.0000,-0.6329,0.0000},
{0.2176,0.0000,0.9981,0.0000,0.2434,0.0000,1.4433,0.0000,-0.0101,0.0000,-0.6571,0.0000},
{0.2223,0.0000,0.9980,0.0000,0.2425,0.0000,1.4595,0.0000,-0.0102,0.0000,-0.6814,0.0000},
{0.2269,0.0000,0.9980,0.0000,0.2415,0.0000,1.4757,0.0000,-0.0102,0.0000,-0.7056,0.0000},
{0.2315,0.0000,0.9980,0.0000,0.2406,0.0000,1.4919,0.0000,-0.0103,0.0000,-0.7298,0.0000},
{0.2361,0.0000,0.9979,0.0000,0.2396,0.0000,1.5081,0.0000,-0.0103,0.0000,-0.7541,0.0000},
{0.2415,0.0000,0.9979,0.0000,0.2380,0.0000,1.5230,0.0000,-0.0103,0.0000,-0.7855,0.0000},
{0.2470,0.0000,0.9979,0.0000,0.2363,0.0000,1.5379,0.0000,-0.0103,0.0000,-0.8168,0.0000},
{0.2524,0.0000,0.9978,0.0000,0.2347,0.0000,1.5528,0.0000,-0.0103,0.0000,-0.8482,0.0000},
{0.2578,0.0000,0.9978,0.0000,0.2330,0.0000,1.5678,0.0000,-0.0103,0.0000,-0.8796,0.0000},
{0.2632,0.0000,0.9978,0.0000,0.2314,0.0000,1.5827,0.0000,-0.0103,0.0000,-0.9110,0.0000},
{0.2686,0.0000,0.9977,0.0000,0.2297,0.0000,1.5976,0.0000,-0.0103,0.0000,-0.9424,0.0000},
{0.2740,0.0000,0.9977,0.0000,0.2281,0.0000,1.6125,0.0000,-0.0103,0.0000,-0.9738,0.0000},
{0.2794,0.0000,0.9977,0.0000,0.2264,0.0000,1.6275,0.0000,-0.0103,0.0000,-1.0052,0.0000},
{0.2848,0.0000,0.9976,0.0000,0.2248,0.0000,1.6424,0.0000,-0.0103,0.0000,-1.0365,0.0000},
{0.2902,0.0000,0.9976,0.0000,0.2231,0.0000,1.6573,0.0000,-0.0103,0.0000,-1.0679,0.0000},
{0.2956,0.0000,0.9976,0.0000,0.2215,0.0000,1.6722,0.0000,-0.0103,0.0000,-1.0993,0.0000},
{0.3010,0.0000,0.9975,0.0000,0.2198,0.0000,1.6872,0.0000,-0.0103,0.0000,-1.1307,0.0000},
{0.3064,0.0000,0.9975,0.0000,0.2182,0.0000,1.7021,0.0000,-0.0103,0.0000,-1.1621,0.0000},
{0.3118,0.0000,0.9975,0.0000,0.2165,0.0000,1.7170,0.0000,-0.0103,0.0000,-1.1935,0.0000},
{0.3172,0.0000,0.9974,0.0000,0.2149,0.0000,1.7319,0.0000,-0.0103,0.0000,-1.2249,0.0000},
{0.3233,0.0000,0.9974,0.0000,0.2123,0.0000,1.7448,0.0000,-0.0102,0.0000,-1.2692,0.0000},
{0.3294,0.0000,0.9974,0.0000,0.2097,0.0000,1.7577,0.0000,-0.0101,0.0000,-1.3135,0.0000},
{0.3356,0.0000,0.9973,0.0000,0.2071,0.0000,1.7706,0.0000,-0.0100,0.0000,-1.3578,0.0000},
{0.3417,0.0000,0.9973,0.0000,0.2045,0.0000,1.7834,0.0000,-0.0100,0.0000,-1.4021,0.0000},
{0.3478,0.0000,0.9973,0.0000,0.2019,0.0000,1.7963,0.0000,-0.0099,0.0000,-1.4464,0.0000},
{0.3539,0.0000,0.9972,0.0000,0.1993,0.0000,1.8092,0.0000,-0.0098,0.0000,-1.4907,0.0000},
{0.3600,0.0000,0.9972,0.0000,0.1967,0.0000,1.8220,0.0000,-0.0098,0.0000,-1.5351,0.0000},
{0.3661,0.0000,0.9972,0.0000,0.1941,0.0000,1.8349,0.0000,-0.0097,0.0000,-1.5794,0.0000},
{0.3722,0.0000,0.9971,0.0000,0.1915,0.0000,1.8478,0.0000,-0.0096,0.0000,-1.6237,0.0000},
{0.3783,0.0000,0.9971,0.0000,0.1889,0.0000,1.8607,0.0000,-0.0095,0.0000,-1.6680,0.0000},
{0.3844,0.0000,0.9971,0.0000,0.1863,0.0000,1.8735,-0.0000,-0.0095,0.0000,-1.7123,0.0000},
{0.3905,0.0000,0.9970,0.0000,0.1837,0.0000,1.8864,-0.0000,-0.0094,0.0000,-1.7566,0.0000},
{0.3966,0.0000,0.9970,0.0000,0.1811,0.0000,1.8993,-0.0000,-0.0093,0.0000,-1.8009,0.0000},
{0.4027,0.0000,0.9970,0.0000,0.1785,0.0000,1.9121,-0.0000,-0.0092,0.0000,-1.8453,0.0000},
{0.4088,0.0000,0.9969,0.0000,0.1759,0.0000,1.9250,-0.0000,-0.0092,0.0000,-1.8896,0.0000},
{0.4155,0.0000,0.9969,0.0000,0.1720,0.0000,1.9347,-0.0000,-0.0090,0.0000,-1.9524,0.0000},
{0.4222,0.0000,0.9969,0.0000,0.1681,0.0000,1.9444,-0.0000,-0.0089,0.0000,-2.0152,0.0000},
{0.4288,0.0000,0.9969,0.0000,0.1641,0.0000,1.9541,-0.0000,-0.0087,-0.0000,-2.0781,0.0000},
{0.4355,0.0000,0.9968,0.0000,0.1602,0.0000,1.9638,-0.0000,-0.0085,-0.0000,-2.1409,0.0000},
{0.4422,0.0000,0.9968,0.0000,0.1563,0.0000,1.9735,-0.0000,-0.0084,-0.0000,-2.2037,0.0000},
{0.4489,0.0000,0.9968,0.0000,0.1523,0.0000,1.9832,-0.0000,-0.0082,-0.0000,-2.2666,0.0000},
{0.4555,0.0000,0.9968,0.0000,0.1484,0.0000,1.9929,-0.0000,-0.0081,-0.0000,-2.3294,0.0000},
{0.4622,0.0000,0.9967,0.0000,0.1445,0.0000,2.0026,-0.0000,-0.0079,-0.0000,-2.3923,0.0000},
{0.4689,0.0000,0.9967,0.0000,0.1405,0.0000,2.0123,-0.0000,-0.0077,-0.0000,-2.4551,0.0000},
{0.4756,0.0000,0.9967,0.0000,0.1366,0.0000,2.0221,-0.0000,-0.0076,-0.0000,-2.5179,0.0000},
{0.4823,0.0000,0.9966,0.0000,0.1327,0.0000,2.0318,-0.0000,-0.0074,-0.0000,-2.5808,0.0000},
{0.4889,0.0000,0.9966,0.0000,0.1287,0.0000,2.0415,-0.0000,-0.0073,-0.0000,-2.6436,0.0000},
{0.4956,0.0000,0.9966,0.0000,0.1248,0.0000,2.0512,-0.0000,-0.0071,-0.0000,-2.7064,0.0000},
{0.5023,0.0000,0.9966,0.0000,0.1209,0.0000,2.0609,-0.0000,-0.0069,-0.0000,-2.7693,0.0000},
{0.5090,0.0000,0.9965,0.0000,0.1169,0.0000,2.0706,-0.0000,-0.0068,-0.0000,-2.8321,0.0000},
{0.5160,0.0000,0.9965,0.0000,0.1111,0.0000,2.0756,-0.0000,-0.0065,-0.0000,-2.9188,0.0000},
{0.5231,0.0000,0.9965,0.0000,0.1053,0.0000,2.0805,-0.0000,-0.0063,-0.0000,-3.0055,-0.0000},
{0.5301,0.0000,0.9965,0.0000,0.0995,0.0000,2.0855,-0.0000,-0.0060,-0.0000,-3.0922,-0.0000},
{0.5372,0.0000,0.9965,0.0000,0.0937,0.0000,2.0905,-0.0000,-0.0057,-0.0000,-3.1788,-0.0000},
{0.5442,0.0000,0.9965,0.0000,0.0879,0.0000,2.0955,-0.0000,-0.0054,-0.0000,-3.2655,-0.0000},
{0.5513,0.0000,0.9964,0.0000,0.0821,0.0000,2.1004,-0.0000,-0.0052,-0.0000,-3.3522,-0.0000},
{0.5583,0.0000,0.9964,0.0000,0.0763,0.0000,2.1054,-0.0000,-0.0049,-0.0000,-3.4389,-0.0000},
{0.5654,0.0000,0.9964,0.0000,0.0705,0.0000,2.1104,-0.0000,-0.0046,-0.0000,-3.5256,-0.0000},
{0.5724,0.0000,0.9964,0.0000,0.0647,0.0000,2.1154,-0.0000,-0.0044,-0.0000,-3.6123,-0.0000},
{0.5795,0.0000,0.9964,0.0000,0.0589,0.0000,2.1203,-0.0000,-0.0041,-0.0000,-3.6989,-0.0000},
{0.5865,0.0000,0.9964,0.0000,0.0531,0.0000,2.1253,-0.0000,-0.0038,-0.0000,-3.7856,-0.0000},
{0.5936,0.0000,0.9964,0.0000,0.0473,0.0000,2.1303,-0.0000,-0.0036,-0.0000,-3.8723,-0.0000},
{0.6006,0.0000,0.9963,0.0000,0.0415,0.0000,2.1353,-0.0000,-0.0033,-0.0000,-3.9590,-0.0000},
{0.6077,0.0000,0.9963,0.0000,0.0357,0.0000,2.1403,-0.0000,-0.0030,-0.0000,-4.0457,-0.0000},
{0.6147,0.0000,0.9963,0.0000,0.0299,0.0000,2.1452,-0.0000,-0.0028,-0.0000,-4.1323,-0.0000},
{0.6218,0.0000,0.9963,0.0000,0.0215,0.0000,2.1433,-0.0000,-0.0024,-0.0000,-4.2478,-0.0000},
{0.6290,0.0000,0.9963,0.0000,0.0132,0.0000,2.1414,-0.0000,-0.0020,-0.0000,-4.3633,-0.0000},
{0.6361,0.0000,0.9963,0.0000,0.0049,0.0000,2.1395,-0.0000,-0.0016,-0.0000,-4.4788,-0.0000},
{0.6433,0.0000,0.9963,0.0000,-0.0035,0.0000,2.1376,-0.0000,-0.0012,-0.0000,-4.5943,-0.0000},
{0.6504,0.0000,0.9963,0.0000,-0.0118,0.0000,2.1357,-0.0000,-0.0008,-0.0000,-4.7098,-0.0000},
{0.6575,0.0000,0.9963,0.0000,-0.0201,0.0000,2.1339,-0.0000,-0.0004,-0.0000,-4.8253,-0.0000},
{0.6647,0.0000,0.9963,0.0000,-0.0285,0.0000,2.1320,-0.0000,0.0000,-0.0000,-4.9408,-0.0000},
{0.6718,0.0000,0.9963,0.0000,-0.0368,0.0000,2.1301,-0.0000,0.0004,-0.0000,-5.0563,-0.0000},
{0.6789,0.0000,0.9963,0.0000,-0.0451,0.0000,2.1282,-0.0000,0.0008,-0.0000,-5.1718,-0.0000},
{0.6861,0.0000,0.9963,0.0000,-0.0534,0.0000,2.1263,-0.0000,0.0012,-0.0000,-5.2873,-0.0000},
{0.6932,0.0000,0.9963,0.0000,-0.0618,0.0000,2.1244,-0.0000,0.0016,-0.0000,-5.4028,-0.0000},
{0.7004,0.0000,0.9963,0.0000,-0.0701,0.0000,2.1225,-0.0000,0.0020,-0.0000,-5.5183,-0.0000},
{0.7075,0.0000,0.9963,0.0000,-0.0784,0.0000,2.1206,-0.0000,0.0024,-0.0000,-5.6338,-0.0000},
{0.7146,0.0000,0.9963,0.0000,-0.0868,0.0000,2.1187,-0.0000,0.0029,-0.0000,-5.7493,-0.0000},
{0.7218,0.0000,0.9963,0.0000,-0.0951,0.0000,2.1168,-0.0000,0.0033,-0.0000,-5.8648,-0.0000},
{0.7286,0.0000,0.9963,0.0000,-0.1067,0.0000,2.1052,-0.0000,0.0038,-0.0000,-6.0136,-0.0000},
{0.7354,0.0000,0.9964,0.0000,-0.1184,0.0000,2.0935,-0.0000,0.0044,-0.0000,-6.1623,-0.0000},
{0.7422,0.0000,0.9964,0.0000,-0.1300,0.0000,2.0819,-0.0000,0.0049,-0.0000,-6.3111,-0.0000},
{0.7490,0.0000,0.9964,0.0000,-0.1416,0.0000,2.0703,-0.0000,0.0055,-0.0000,-6.4599,-0.0000},
{0.7558,0.0000,0.9964,0.0000,-0.1533,0.0000,2.0587,-0.0000,0.0061,-0.0000,-6.6087,-0.0000},
{0.7626,0.0000,0.9965,-0.0000,-0.1649,0.0000,2.0470,-0.0000,0.0066,-0.0000,-6.7575,-0.0000},
{0.7695,0.0000,0.9965,-0.0000,-0.1765,0.0000,2.0354,-0.0000,0.0072,-0.0000,-6.9063,-0.0000},
{0.7763,0.0000,0.9965,-0.0000,-0.1882,0.0000,2.0238,-0.0000,0.0077,-0.0000,-7.0551,-0.0000},
{0.7831,0.0000,0.9965,-0.0000,-0.1998,0.0000,2.0122,-0.0000,0.0083,-0.0000,-7.2039,-0.0000},
{0.7899,0.0000,0.9966,-0.0000,-0.2114,0.0000,2.0006,-0.0000,0.0089,-0.0000,-7.3527,-0.0000},
{0.7967,0.0000,0.9966,-0.0000,-0.2231,0.0000,1.9889,-0.0000,0.0094,-0.0000,-7.5015,-0.0000},
{0.8035,0.0000,0.9966,-0.0000,-0.2347,0.0000,1.9773,-0.0000,0.0100,-0.0000,-7.6503,-0.0000},
{0.8103,0.0000,0.9966,-0.0000,-0.2463,0.0000,1.9657,-0.0000,0.0105,-0.0000,-7.7991,-0.0000},
{0.8171,0.0000,0.9967,-0.0000,-0.2580,0.0000,1.9541,-0.0000,0.0111,-0.0000,-7.9479,-0.0000},
{0.8240,0.0000,0.9967,-0.0000,-0.2696,0.0000,1.9424,-0.0000,0.0117,-0.0000,-8.0966,-0.0000},
{0.8299,0.0000,0.9967,-0.0000,-0.2854,0.0000,1.9174,-0.0000,0.0124,-0.0000,-8.2826,-0.0000},
{0.8358,0.0000,0.9968,-0.0000,-0.3013,0.0000,1.8924,-0.0000,0.0132,-0.0000,-8.4686,-0.0000},
{0.8417,0.0000,0.9969,-0.0000,-0.3171,0.0000,1.8674,-0.0000,0.0139,-0.0000,-8.6546,-0.0000},
{0.8476,0.0000,0.9969,-0.0000,-0.3329,0.0000,1.8424,-0.0000,0.0147,-0.0000,-8.8406,-0.0000},
{0.8535,0.0000,0.9970,-0.0000,-0.3487,0.0000,1.8174,-0.0000,0.0154,-0.0000,-9.0265,-0.0000},
{0.8594,0.0000,0.9970,-0.0000,-0.3645,0.0000,1.7924,-0.0000,0.0162,-0.0000,-9.2125,-0.0000},
{0.8654,0.0000,0.9971,-0.0000,-0.3803,0.0000,1.7674,-0.0000,0.0169,-0.0000,-9.3985,-0.0000},
{0.8713,0.0000,0.9971,-0.0000,-0.3962,0.0000,1.7424,-0.0000,0.0177,-0.0000,-9.5845,-0.0000},
{0.8772,0.0000,0.9972,-0.0000,-0.4120,0.0000,1.7173,-0.0000,0.0184,0.0000,-9.7705,-0.0000},
{0.8831,0.0000,0.9973,-0.0000,-0.4278,0.0000,1.6923,-0.0000,0.0192,0.0000,-9.9564,-0.0000},
{0.8890,0.0000,0.9973,-0.0000,-0.4436,0.0000,1.6673,-0.0000,0.0199,0.0000,-10.1424,-0.0000},
{0.8949,0.0000,0.9974,-0.0000,-0.4594,0.0000,1.6423,-0.0000,0.0207,0.0000,-10.3284,-0.0000},
{0.9008,0.0000,0.9974,-0.0000,-0.4753,0.0000,1.6173,-0.0000,0.0214,0.0000,-10.5144,-0.0000},
{0.9068,0.0000,0.9975,-0.0000,-0.4911,0.0000,1.5923,-0.0000,0.0222,0.0000,-10.7004,-0.0000},
{0.9127,0.0000,0.9976,-0.0000,-0.5069,0.0000,1.5673,-0.0000,0.0229,0.0000,-10.8863,-0.0000},
{0.9169,0.0000,0.9977,-0.0000,-0.5279,0.0000,1.5243,-0.0000,0.0239,0.0000,-11.1126,-0.0000},
{0.9211,0.0000,0.9978,-0.0000,-0.5488,0.0000,1.4813,-0.0000,0.0249,0.0000,-11.3389,-0.0000},
{0.9254,0.0000,0.9979,-0.0000,-0.5698,-0.0000,1.4383,-0.0000,0.0258,0.0000,-11.5652,-0.0000},
{0.9296,0.0000,0.9980,-0.0000,-0.5908,-0.0000,1.3954,-0.0000,0.0268,0.0000,-11.7915,-0.0000},
{0.9338,0.0000,0.9981,-0.0000,-0.6118,-0.0000,1.3524,-0.0000,0.0278,0.0000,-12.0178,-0.0000},
{0.9381,0.0000,0.9982,-0.0000,-0.6327,-0.0000,1.3094,-0.0000,0.0287,0.0000,-12.2441,-0.0000},
{0.9423,0.0000,0.9983,-0.0000,-0.6537,-0.0000,1.2664,-0.0000,0.0297,0.0000,-12.4704,-0.0000},
{0.9466,0.0000,0.9984,-0.0000,-0.6747,-0.0000,1.2235,-0.0000,0.0307,0.0000,-12.6967,-0.0000},
{0.9508,0.0000,0.9985,-0.0000,-0.6957,-0.0000,1.1805,-0.0000,0.0317,0.0000,-12.9230,-0.0000},
{0.9550,0.0000,0.9986,-0.0000,-0.7166,-0.0000,1.1375,-0.0000,0.0326,0.0000,-13.1493,0.0000},
{0.9593,0.0000,0.9987,-0.0000,-0.7376,-0.0000,1.0945,-0.0000,0.0336,0.0000,-13.3755,0.0000},
{0.9635,0.0000,0.9988,-0.0000,-0.7586,-0.0000,1.0516,-0.0000,0.0346,0.0000,-13.6018,0.0000},
{0.9677,0.0000,0.9989,-0.0000,-0.7795,-0.0000,1.0086,-0.0000,0.0355,0.0000,-13.8281,0.0000},
{0.9720,0.0000,0.9990,-0.0000,-0.8005,-0.0000,0.9656,-0.0000,0.0365,0.0000,-14.0544,0.0000},
{0.9762,0.0000,0.9991,-0.0000,-0.8215,-0.0000,0.9226,-0.0000,0.0375,0.0000,-14.2807,0.0000},
{0.9778,0.0000,0.9991,-0.0000,-0.8334,-0.0000,0.8611,-0.0000,0.0350,0.0000,-13.3287,0.0000},
{0.9794,0.0000,0.9992,-0.0000,-0.8453,-0.0000,0.7996,-0.0000,0.0325,0.0000,-12.3766,0.0000},
{0.9810,0.0000,0.9993,-0.0000,-0.8572,-0.0000,0.7381,-0.0000,0.0300,0.0000,-11.4246,0.0000},
{0.9826,-0.0000,0.9993,-0.0000,-0.8691,-0.0000,0.6766,-0.0000,0.0275,0.0000,-10.4725,0.0000},
{0.9841,-0.0000,0.9994,-0.0000,-0.8810,-0.0000,0.6151,-0.0000,0.0250,0.0000,-9.5205,0.0000},
{0.9857,-0.0000,0.9994,-0.0000,-0.8929,-0.0000,0.5536,-0.0000,0.0225,0.0000,-8.5684,0.0000},
{0.9873,-0.0000,0.9995,-0.0000,-0.9048,-0.0000,0.4921,-0.0000,0.0200,0.0000,-7.6164,0.0000},
{0.9889,-0.0000,0.9996,-0.0000,-0.9167,-0.0000,0.4306,-0.0000,0.0175,0.0000,-6.6643,0.0000},
{0.9905,-0.0000,0.9996,-0.0000,-0.9286,-0.0000,0.3691,-0.0000,0.0150,0.0000,-5.7123,0.0000},
{0.9921,-0.0000,0.9997,-0.0000,-0.9405,-0.0000,0.3075,-0.0000,0.0125,0.0000,-4.7602,0.0000},
{0.9937,-0.0000,0.9998,-0.0000,-0.9524,-0.0000,0.2460,-0.0000,0.0100,0.0000,-3.8082,0.0000},
{0.9952,-0.0000,0.9998,-0.0000,-0.9643,-0.0000,0.1845,-0.0000,0.0075,0.0000,-2.8561,0.0000},
{0.9968,-0.0000,0.9999,-0.0000,-0.9762,-0.0000,0.1230,-0.0000,0.0050,0.0000,-1.9041,0.0000},
{0.9984,-0.0000,0.9999,-0.0000,-0.9881,-0.0000,0.0615,-0.0000,0.0025,0.0000,-0.9520,0.0000},
{1.0000,-0.0000,1.0000,0.0000,-1.0000,-0.0000,-0.0000,0.0000,0.0000,0.0000,-0.0000,0.0000}};

static const float U_ref_data[255][4] = {
{-0.0779,0.0762,0.0762,-0.0779},
{-0.0710,0.0692,0.0692,-0.0710},
{-0.0641,0.0623,0.0623,-0.0641},
{-0.0572,0.0553,0.0553,-0.0572},
{-0.0503,0.0483,0.0483,-0.0503},
{-0.0435,0.0414,0.0414,-0.0435},
{-0.0366,0.0344,0.0344,-0.0366},
{-0.0297,0.0275,0.0275,-0.0297},
{-0.0228,0.0205,0.0205,-0.0228},
{-0.0159,0.0135,0.0135,-0.0159},
{-0.0090,0.0066,0.0066,-0.0090},
{-0.0021,-0.0004,-0.0004,-0.0021},
{0.0048,-0.0074,-0.0074,0.0048},
{0.0116,-0.0143,-0.0143,0.0116},
{0.0185,-0.0213,-0.0213,0.0185},
{0.0254,-0.0283,-0.0283,0.0254},
{0.0249,-0.0278,-0.0278,0.0249},
{0.0244,-0.0274,-0.0274,0.0244},
{0.0239,-0.0269,-0.0269,0.0239},
{0.0234,-0.0265,-0.0265,0.0234},
{0.0229,-0.0260,-0.0260,0.0229},
{0.0224,-0.0256,-0.0256,0.0224},
{0.0220,-0.0252,-0.0252,0.0220},
{0.0215,-0.0247,-0.0247,0.0215},
{0.0210,-0.0243,-0.0243,0.0210},
{0.0205,-0.0238,-0.0238,0.0205},
{0.0200,-0.0234,-0.0234,0.0200},
{0.0195,-0.0229,-0.0229,0.0195},
{0.0190,-0.0225,-0.0225,0.0190},
{0.0185,-0.0221,-0.0221,0.0185},
{0.0180,-0.0216,-0.0216,0.0180},
{0.0176,-0.0212,-0.0212,0.0176},
{0.0172,-0.0209,-0.0209,0.0172},
{0.0168,-0.0205,-0.0205,0.0168},
{0.0164,-0.0202,-0.0202,0.0164},
{0.0160,-0.0198,-0.0198,0.0160},
{0.0156,-0.0194,-0.0194,0.0156},
{0.0152,-0.0191,-0.0191,0.0152},
{0.0149,-0.0187,-0.0187,0.0149},
{0.0145,-0.0183,-0.0183,0.0145},
{0.0141,-0.0180,-0.0180,0.0141},
{0.0137,-0.0176,-0.0176,0.0137},
{0.0133,-0.0172,-0.0172,0.0133},
{0.0129,-0.0169,-0.0169,0.0129},
{0.0125,-0.0165,-0.0165,0.0125},
{0.0121,-0.0162,-0.0162,0.0121},
{0.0118,-0.0159,-0.0159,0.0118},
{0.0115,-0.0156,-0.0156,0.0115},
{0.0113,-0.0153,-0.0153,0.0113},
{0.0110,-0.0150,-0.0150,0.0110},
{0.0107,-0.0147,-0.0147,0.0107},
{0.0104,-0.0145,-0.0145,0.0104},
{0.0101,-0.0142,-0.0142,0.0101},
{0.0099,-0.0139,-0.0139,0.0099},
{0.0096,-0.0136,-0.0136,0.0096},
{0.0093,-0.0133,-0.0133,0.0093},
{0.0090,-0.0131,-0.0131,0.0090},
{0.0087,-0.0128,-0.0128,0.0087},
{0.0084,-0.0125,-0.0125,0.0084},
{0.0082,-0.0122,-0.0122,0.0082},
{0.0079,-0.0119,-0.0119,0.0079},
{0.0077,-0.0117,-0.0117,0.0077},
{0.0075,-0.0116,-0.0116,0.0075},
{0.0074,-0.0114,-0.0114,0.0074},
{0.0072,-0.0112,-0.0112,0.0072},
{0.0070,-0.0110,-0.0110,0.0070},
{0.0068,-0.0108,-0.0108,0.0068},
{0.0067,-0.0106,-0.0106,0.0067},
{0.0065,-0.0104,-0.0104,0.0065},
{0.0063,-0.0102,-0.0102,0.0063},
{0.0062,-0.0100,-0.0100,0.0062},
{0.0060,-0.0098,-0.0098,0.0060},
{0.0058,-0.0096,-0.0096,0.0058},
{0.0057,-0.0094,-0.0094,0.0057},
{0.0055,-0.0092,-0.0092,0.0055},
{0.0053,-0.0090,-0.0090,0.0053},
{0.0053,-0.0089,-0.0089,0.0053},
{0.0052,-0.0088,-0.0088,0.0052},
{0.0052,-0.0087,-0.0087,0.0052},
{0.0051,-0.0086,-0.0086,0.0051},
{0.0050,-0.0085,-0.0085,0.0050},
{0.0050,-0.0084,-0.0084,0.0050},
{0.0049,-0.0083,-0.0083,0.0049},
{0.0049,-0.0082,-0.0082,0.0049},
{0.0048,-0.0081,-0.0081,0.0048},
{0.0048,-0.0080,-0.0080,0.0048},
{0.0047,-0.0079,-0.0079,0.0047},
{0.0047,-0.0078,-0.0078,0.0047},
{0.0046,-0.0076,-0.0076,0.0046},
{0.0046,-0.0075,-0.0075,0.0046},
{0.0045,-0.0074,-0.0074,0.0045},
{0.0046,-0.0074,-0.0074,0.0046},
{0.0046,-0.0074,-0.0074,0.0046},
{0.0047,-0.0074,-0.0074,0.0047},
{0.0048,-0.0074,-0.0074,0.0048},
{0.0048,-0.0073,-0.0073,0.0048},
{0.0049,-0.0073,-0.0073,0.0049},
{0.0050,-0.0073,-0.0073,0.0050},
{0.0050,-0.0073,-0.0073,0.0050},
{0.0051,-0.0073,-0.0073,0.0051},
{0.0052,-0.0072,-0.0072,0.0052},
{0.0052,-0.0072,-0.0072,0.0052},
{0.0053,-0.0072,-0.0072,0.0053},
{0.0053,-0.0072,-0.0072,0.0053},
{0.0054,-0.0072,-0.0072,0.0054},
{0.0055,-0.0071,-0.0071,0.0055},
{0.0057,-0.0072,-0.0072,0.0057},
{0.0058,-0.0073,-0.0073,0.0058},
{0.0060,-0.0073,-0.0073,0.0060},
{0.0062,-0.0074,-0.0074,0.0062},
{0.0064,-0.0074,-0.0074,0.0064},
{0.0066,-0.0075,-0.0075,0.0066},
{0.0068,-0.0076,-0.0076,0.0068},
{0.0069,-0.0076,-0.0076,0.0069},
{0.0071,-0.0077,-0.0077,0.0071},
{0.0073,-0.0078,-0.0078,0.0073},
{0.0075,-0.0078,-0.0078,0.0075},
{0.0077,-0.0079,-0.0079,0.0077},
{0.0079,-0.0080,-0.0080,0.0079},
{0.0081,-0.0080,-0.0080,0.0081},
{0.0082,-0.0081,-0.0081,0.0082},
{0.0085,-0.0082,-0.0082,0.0085},
{0.0089,-0.0084,-0.0084,0.0089},
{0.0092,-0.0085,-0.0085,0.0092},
{0.0095,-0.0087,-0.0087,0.0095},
{0.0098,-0.0088,-0.0088,0.0098},
{0.0101,-0.0089,-0.0089,0.0101},
{0.0104,-0.0091,-0.0091,0.0104},
{0.0107,-0.0092,-0.0092,0.0107},
{0.0110,-0.0094,-0.0094,0.0110},
{0.0113,-0.0095,-0.0095,0.0113},
{0.0116,-0.0097,-0.0097,0.0116},
{0.0119,-0.0098,-0.0098,0.0119},
{0.0122,-0.0100,-0.0100,0.0122},
{0.0125,-0.0101,-0.0101,0.0125},
{0.0128,-0.0102,-0.0102,0.0128},
{0.0132,-0.0105,-0.0105,0.0132},
{0.0137,-0.0107,-0.0107,0.0137},
{0.0141,-0.0109,-0.0109,0.0141},
{0.0145,-0.0111,-0.0111,0.0145},
{0.0149,-0.0113,-0.0113,0.0149},
{0.0154,-0.0115,-0.0115,0.0154},
{0.0158,-0.0118,-0.0118,0.0158},
{0.0162,-0.0120,-0.0120,0.0162},
{0.0166,-0.0122,-0.0122,0.0166},
{0.0171,-0.0124,-0.0124,0.0171},
{0.0175,-0.0126,-0.0126,0.0175},
{0.0179,-0.0129,-0.0129,0.0179},
{0.0183,-0.0131,-0.0131,0.0183},
{0.0188,-0.0133,-0.0133,0.0188},
{0.0192,-0.0135,-0.0135,0.0192},
{0.0197,-0.0138,-0.0138,0.0197},
{0.0203,-0.0141,-0.0141,0.0203},
{0.0208,-0.0144,-0.0144,0.0208},
{0.0214,-0.0146,-0.0146,0.0214},
{0.0219,-0.0149,-0.0149,0.0219},
{0.0224,-0.0152,-0.0152,0.0224},
{0.0230,-0.0155,-0.0155,0.0230},
{0.0235,-0.0158,-0.0158,0.0235},
{0.0241,-0.0161,-0.0161,0.0241},
{0.0246,-0.0163,-0.0163,0.0246},
{0.0252,-0.0166,-0.0166,0.0252},
{0.0257,-0.0169,-0.0169,0.0257},
{0.0262,-0.0172,-0.0172,0.0262},
{0.0268,-0.0175,-0.0175,0.0268},
{0.0273,-0.0178,-0.0178,0.0273},
{0.0280,-0.0181,-0.0181,0.0280},
{0.0286,-0.0184,-0.0184,0.0286},
{0.0293,-0.0188,-0.0188,0.0293},
{0.0300,-0.0191,-0.0191,0.0300},
{0.0306,-0.0195,-0.0195,0.0306},
{0.0313,-0.0198,-0.0198,0.0313},
{0.0319,-0.0202,-0.0202,0.0319},
{0.0326,-0.0205,-0.0205,0.0326},
{0.0333,-0.0208,-0.0208,0.0333},
{0.0339,-0.0212,-0.0212,0.0339},
{0.0346,-0.0215,-0.0215,0.0346},
{0.0352,-0.0219,-0.0219,0.0352},
{0.0359,-0.0222,-0.0222,0.0359},
{0.0365,-0.0225,-0.0225,0.0365},
{0.0372,-0.0229,-0.0229,0.0372},
{0.0380,-0.0233,-0.0233,0.0380},
{0.0387,-0.0237,-0.0237,0.0387},
{0.0395,-0.0240,-0.0240,0.0395},
{0.0403,-0.0244,-0.0244,0.0403},
{0.0410,-0.0248,-0.0248,0.0410},
{0.0418,-0.0252,-0.0252,0.0418},
{0.0426,-0.0256,-0.0256,0.0426},
{0.0433,-0.0260,-0.0260,0.0433},
{0.0441,-0.0264,-0.0264,0.0441},
{0.0449,-0.0268,-0.0268,0.0449},
{0.0456,-0.0271,-0.0271,0.0456},
{0.0464,-0.0275,-0.0275,0.0464},
{0.0472,-0.0279,-0.0279,0.0472},
{0.0479,-0.0283,-0.0283,0.0479},
{0.0487,-0.0287,-0.0287,0.0487},
{0.0496,-0.0291,-0.0291,0.0496},
{0.0505,-0.0295,-0.0295,0.0505},
{0.0513,-0.0300,-0.0300,0.0513},
{0.0522,-0.0304,-0.0304,0.0522},
{0.0531,-0.0308,-0.0308,0.0531},
{0.0539,-0.0312,-0.0312,0.0539},
{0.0548,-0.0316,-0.0316,0.0548},
{0.0557,-0.0321,-0.0321,0.0557},
{0.0565,-0.0325,-0.0325,0.0565},
{0.0574,-0.0329,-0.0329,0.0574},
{0.0583,-0.0333,-0.0333,0.0583},
{0.0591,-0.0337,-0.0337,0.0591},
{0.0600,-0.0342,-0.0342,0.0600},
{0.0609,-0.0346,-0.0346,0.0609},
{0.0618,-0.0350,-0.0350,0.0618},
{0.0627,-0.0354,-0.0354,0.0627},
{0.0637,-0.0359,-0.0359,0.0637},
{0.0646,-0.0363,-0.0363,0.0646},
{0.0656,-0.0367,-0.0367,0.0656},
{0.0666,-0.0372,-0.0372,0.0666},
{0.0675,-0.0376,-0.0376,0.0675},
{0.0685,-0.0381,-0.0381,0.0685},
{0.0694,-0.0385,-0.0385,0.0694},
{0.0704,-0.0389,-0.0389,0.0704},
{0.0714,-0.0394,-0.0394,0.0714},
{0.0723,-0.0398,-0.0398,0.0723},
{0.0733,-0.0402,-0.0402,0.0733},
{0.0743,-0.0407,-0.0407,0.0743},
{0.0752,-0.0411,-0.0411,0.0752},
{0.0762,-0.0416,-0.0416,0.0762},
{0.0516,-0.0252,-0.0252,0.0516},
{0.0271,-0.0089,-0.0089,0.0271},
{0.0025,0.0074,0.0074,0.0025},
{-0.0221,0.0237,0.0237,-0.0221},
{-0.0466,0.0400,0.0400,-0.0466},
{-0.0712,0.0563,0.0563,-0.0712},
{-0.0957,0.0726,0.0726,-0.0957},
{-0.1203,0.0889,0.0889,-0.1203},
{-0.1449,0.1052,0.1052,-0.1449},
{-0.1694,0.1215,0.1215,-0.1694},
{-0.1940,0.1378,0.1378,-0.1940},
{-0.2185,0.1541,0.1541,-0.2185},
{-0.2431,0.1705,0.1705,-0.2431},
{-0.2677,0.1868,0.1868,-0.2677},
{-0.2922,0.2031,0.2031,-0.2922},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000},
{0.0000,0.0000,0.0000,0.0000}};
