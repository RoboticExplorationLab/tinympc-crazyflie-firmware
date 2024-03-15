#pragma once

#include <tinympc/types.hpp>

static const tinytype Xref_data[3*(NTOTAL+ADDITIONAL-1)]  = {
	0.000000f, 1.000000f, 0.998752f, 
	0.022470f, 0.999748f, 0.997503f, 
	0.044929f, 0.998990f, 0.996255f, 
	0.067365f, 0.997728f, 0.995006f, 
	0.089767f, 0.995963f, 0.993758f, 
	0.112123f, 0.993694f, 0.992509f, 
	0.134423f, 0.990924f, 0.991261f, 
	0.156655f, 0.987653f, 0.990012f, 
	0.178808f, 0.983884f, 0.988764f, 
	0.200871f, 0.979618f, 0.987516f, 
	0.222833f, 0.974857f, 0.986267f, 
	0.244681f, 0.969604f, 0.985019f, 
	0.266407f, 0.963861f, 0.983770f, 
	0.287997f, 0.957631f, 0.982522f, 
	0.309443f, 0.950918f, 0.981273f, 
	0.330732f, 0.943725f, 0.980025f, 
	0.351854f, 0.936055f, 0.978777f, 
	0.372798f, 0.927913f, 0.977528f, 
	0.393554f, 0.919302f, 0.976280f, 
	0.414111f, 0.910226f, 0.975031f, 
	0.434460f, 0.900691f, 0.973783f, 
	0.454588f, 0.890702f, 0.972534f, 
	0.474488f, 0.880262f, 0.971286f, 
	0.494147f, 0.869378f, 0.970037f, 
	0.513558f, 0.858055f, 0.968789f, 
	0.532708f, 0.846299f, 0.967541f, 
	0.551590f, 0.834115f, 0.966292f, 
	0.570194f, 0.821510f, 0.965044f, 
	0.588509f, 0.808491f, 0.963795f, 
	0.606527f, 0.795063f, 0.962547f, 
	0.624239f, 0.781233f, 0.961298f, 
	0.641636f, 0.767009f, 0.960050f, 
	0.658709f, 0.752398f, 0.958801f, 
	0.675449f, 0.737407f, 0.957553f, 
	0.691848f, 0.722043f, 0.956305f, 
	0.707897f, 0.706315f, 0.955056f, 
	0.723590f, 0.690231f, 0.953808f, 
	0.738916f, 0.673797f, 0.952559f, 
	0.753870f, 0.657024f, 0.951311f, 
	0.768443f, 0.639918f, 0.950062f, 
	0.782628f, 0.622490f, 0.948814f, 
	0.796418f, 0.604747f, 0.947566f, 
	0.809805f, 0.586699f, 0.946317f, 
	0.822784f, 0.568354f, 0.945069f, 
	0.835347f, 0.549723f, 0.943820f, 
	0.847489f, 0.530814f, 0.942572f, 
	0.859202f, 0.511637f, 0.941323f, 
	0.870482f, 0.492201f, 0.940075f, 
	0.881322f, 0.472517f, 0.938826f, 
	0.891716f, 0.452594f, 0.937578f, 
	0.901661f, 0.432443f, 0.936330f, 
	0.911150f, 0.412074f, 0.935081f, 
	0.920180f, 0.391496f, 0.933833f, 
	0.928744f, 0.370721f, 0.932584f, 
	0.936840f, 0.349758f, 0.931336f, 
	0.944462f, 0.328619f, 0.930087f, 
	0.951608f, 0.307314f, 0.928839f, 
	0.958273f, 0.285854f, 0.927591f, 
	0.964454f, 0.264249f, 0.926342f, 
	0.970149f, 0.242511f, 0.925094f, 
	0.975353f, 0.220651f, 0.923845f, 
	0.980065f, 0.198679f, 0.922597f, 
	0.984281f, 0.176607f, 0.921348f, 
	0.988001f, 0.154445f, 0.920100f, 
	0.991222f, 0.132206f, 0.918851f, 
	0.993943f, 0.109900f, 0.917603f, 
	0.996161f, 0.087538f, 0.916355f, 
	0.997877f, 0.065132f, 0.915106f, 
	0.999088f, 0.042693f, 0.913858f, 
	0.999795f, 0.020233f, 0.912609f, 
	0.999997f, -0.002237f, 0.911361f, 
	0.999695f, -0.024707f, 0.910112f, 
	0.998887f, -0.047164f, 0.908864f, 
	0.997575f, -0.069597f, 0.907615f, 
	0.995759f, -0.091995f, 0.906367f, 
	0.993441f, -0.114346f, 0.905119f, 
	0.990621f, -0.136640f, 0.903870f, 
	0.987300f, -0.158865f, 0.902622f, 
	0.983481f, -0.181009f, 0.901373f, 
	0.979166f, -0.203062f, 0.900125f, 
	0.974356f, -0.225013f, 0.898876f, 
	0.969054f, -0.246850f, 0.897628f, 
	0.963262f, -0.268562f, 0.896380f, 
	0.956984f, -0.290139f, 0.895131f, 
	0.950223f, -0.311569f, 0.893883f, 
	0.942983f, -0.332842f, 0.892634f, 
	0.935265f, -0.353947f, 0.891386f, 
	0.927076f, -0.374873f, 0.890137f, 
	0.918419f, -0.395610f, 0.888889f, 
	0.909297f, -0.416147f, 0.887640f, 
	0.899717f, -0.436474f, 0.886392f, 
	0.889682f, -0.456580f, 0.885144f, 
	0.879198f, -0.476456f, 0.883895f, 
	0.868270f, -0.496091f, 0.882647f, 
	0.856904f, -0.515476f, 0.881398f, 
	0.845105f, -0.534601f, 0.880150f, 
	0.832879f, -0.553455f, 0.878901f, 
	0.820233f, -0.572030f, 0.877653f, 
	0.807172f, -0.590316f, 0.876404f, 
	0.793704f, -0.608305f, 0.875156f, 
	0.779835f, -0.625986f, 0.873908f, 
	0.765572f, -0.643350f, 0.872659f, 
	0.750922f, -0.660390f, 0.871411f, 
	0.735894f, -0.677097f, 0.870162f, 
	0.720494f, -0.693461f, 0.868914f, 
	0.704730f, -0.709476f, 0.867665f, 
	0.688610f, -0.725132f, 0.866417f, 
	0.672142f, -0.740422f, 0.865169f, 
	0.655335f, -0.755338f, 0.863920f, 
	0.638197f, -0.769873f, 0.862672f, 
	0.620737f, -0.784019f, 0.861423f, 
	0.602963f, -0.797769f, 0.860175f, 
	0.584885f, -0.811116f, 0.858926f, 
	0.566512f, -0.824054f, 0.857678f, 
	0.547852f, -0.836575f, 0.856429f, 
	0.528916f, -0.848674f, 0.855181f, 
	0.509713f, -0.860345f, 0.853933f, 
	0.490252f, -0.871581f, 0.852684f, 
	0.470544f, -0.882377f, 0.851436f, 
	0.450598f, -0.892727f, 0.850187f, 
	0.430425f, -0.902626f, 0.848939f, 
	0.410034f, -0.912070f, 0.847690f, 
	0.389436f, -0.921053f, 0.846442f, 
	0.368642f, -0.929571f, 0.845194f, 
	0.347661f, -0.937620f, 0.843945f, 
	0.326505f, -0.945195f, 0.842697f, 
	0.305184f, -0.952293f, 0.841448f, 
	0.283709f, -0.958910f, 0.840200f, 
	0.262091f, -0.965043f, 0.838951f, 
	0.240340f, -0.970689f, 0.837703f, 
	0.218468f, -0.975844f, 0.836454f, 
	0.196486f, -0.980507f, 0.835206f, 
	0.174404f, -0.984674f, 0.833958f, 
	0.152234f, -0.988344f, 0.832709f, 
	0.129988f, -0.991516f, 0.831461f, 
	0.107676f, -0.994186f, 0.830212f, 
	0.085309f, -0.996355f, 0.828964f, 
	0.062899f, -0.998020f, 0.827715f, 
	0.040458f, -0.999181f, 0.826467f, 
	0.017996f, -0.999838f, 0.825218f, 
	-0.004475f, -0.999990f, 0.823970f, 
	-0.026943f, -0.999637f, 0.822722f, 
	-0.049398f, -0.998779f, 0.821473f, 
	-0.071829f, -0.997417f, 0.820225f, 
	-0.094222f, -0.995551f, 0.818976f, 
	-0.116569f, -0.993183f, 0.817728f, 
	-0.138856f, -0.990313f, 0.816479f, 
	-0.161073f, -0.986942f, 0.815231f, 
	-0.183209f, -0.983074f, 0.813983f, 
	-0.205253f, -0.978709f, 0.812734f, 
	-0.227193f, -0.973850f, 0.811486f, 
	-0.249018f, -0.968499f, 0.810237f, 
	-0.270717f, -0.962659f, 0.808989f, 
	-0.292280f, -0.956333f, 0.807740f, 
	-0.313695f, -0.949524f, 0.806492f, 
	-0.334951f, -0.942235f, 0.805243f, 
	-0.356039f, -0.934471f, 0.803995f, 
	-0.376946f, -0.926235f, 0.802747f, 
	-0.397664f, -0.917531f, 0.801498f, 
	-0.418180f, -0.908364f, 0.800250f, 
	-0.438486f, -0.898738f, 0.799001f, 
	-0.458570f, -0.888659f, 0.797753f, 
	-0.478422f, -0.878130f, 0.796504f, 
	-0.498033f, -0.867158f, 0.795256f, 
	-0.517392f, -0.855748f, 0.794007f, 
	-0.536490f, -0.843907f, 0.792759f, 
	-0.555317f, -0.831639f, 0.791511f, 
	-0.573864f, -0.818951f, 0.790262f, 
	-0.592121f, -0.805849f, 0.789014f, 
	-0.610079f, -0.792341f, 0.787765f, 
	-0.627729f, -0.778432f, 0.786517f, 
	-0.645062f, -0.764131f, 0.785268f, 
	-0.662069f, -0.749443f, 0.784020f, 
	-0.678742f, -0.734377f, 0.782772f, 
	-0.695072f, -0.718940f, 0.781523f, 
	-0.711051f, -0.703141f, 0.780275f, 
	-0.726671f, -0.686986f, 0.779026f, 
	-0.741924f, -0.670484f, 0.777778f, 
	-0.756802f, -0.653644f, 0.776529f, 
	-0.771299f, -0.636473f, 0.775281f, 
	-0.785406f, -0.618981f, 0.774032f, 
	-0.799116f, -0.601177f, 0.772784f, 
	-0.812423f, -0.583069f, 0.771536f, 
	-0.825319f, -0.564667f, 0.770287f, 
	-0.837799f, -0.545979f, 0.769039f, 
	-0.849855f, -0.527016f, 0.767790f, 
	-0.861483f, -0.507787f, 0.766542f, 
	-0.872675f, -0.488301f, 0.765293f, 
	-0.883427f, -0.468569f, 0.764045f, 
	-0.893733f, -0.448600f, 0.762797f, 
	-0.903587f, -0.428404f, 0.761548f, 
	-0.912985f, -0.407993f, 0.760300f, 
	-0.921922f, -0.387375f, 0.759051f, 
	-0.930394f, -0.366561f, 0.757803f, 
	-0.938396f, -0.345563f, 0.756554f, 
	-0.945924f, -0.324390f, 0.755306f, 
	-0.952974f, -0.303053f, 0.754057f, 
	-0.959543f, -0.281563f, 0.752809f, 
	-0.965627f, -0.259931f, 0.751561f, 
	-0.971224f, -0.238168f, 0.750312f, 
	-0.976330f, -0.216284f, 0.749064f, 
	-0.980944f, -0.194291f, 0.747815f, 
	-0.985062f, -0.172201f, 0.746567f, 
	-0.988683f, -0.150023f, 0.745318f, 
	-0.991804f, -0.127769f, 0.744070f, 
	-0.994425f, -0.105451f, 0.742821f, 
	-0.996543f, -0.083080f, 0.741573f, 
	-0.998158f, -0.060666f, 0.740325f, 
	-0.999269f, -0.038222f, 0.739076f, 
	-0.999876f, -0.015759f, 0.737828f, 
	-0.999977f, 0.006712f, 0.736579f, 
	-0.999574f, 0.029180f, 0.735331f, 
	-0.998666f, 0.051633f, 0.734082f, 
	-0.997254f, 0.074060f, 0.732834f, 
	-0.995338f, 0.096450f, 0.731586f, 
	-0.992919f, 0.118791f, 0.730337f, 
	-0.989999f, 0.141071f, 0.729089f, 
	-0.986580f, 0.163281f, 0.727840f, 
	-0.982662f, 0.185408f, 0.726592f, 
	-0.978247f, 0.207442f, 0.725343f, 
	-0.973339f, 0.229371f, 0.724095f, 
	-0.967939f, 0.251184f, 0.722846f, 
	-0.962051f, 0.272870f, 0.721598f, 
	-0.955677f, 0.294418f, 0.720350f, 
	-0.948820f, 0.315818f, 0.719101f, 
	-0.941484f, 0.337058f, 0.717853f, 
	-0.933672f, 0.358129f, 0.716604f, 
	-0.925389f, 0.379018f, 0.715356f, 
	-0.916639f, 0.399716f, 0.714107f, 
	-0.907426f, 0.420212f, 0.712859f, 
	-0.897755f, 0.440495f, 0.711610f, 
	-0.887630f, 0.460557f, 0.710362f, 
	-0.877057f, 0.480385f, 0.709114f, 
	-0.866042f, 0.499972f, 0.707865f, 
	-0.854589f, 0.519305f, 0.706617f, 
	-0.842704f, 0.538377f, 0.705368f, 
	-0.830394f, 0.557177f, 0.704120f, 
	-0.817665f, 0.575695f, 0.702871f, 
	-0.804522f, 0.593922f, 0.701623f, 
	-0.790974f, 0.611850f, 0.700375f, 
	-0.777026f, 0.629469f, 0.699126f, 
	-0.762685f, 0.646770f, 0.697878f, 
	-0.747960f, 0.663744f, 0.696629f, 
	-0.732857f, 0.680383f, 0.695381f, 
	-0.717383f, 0.696679f, 0.694132f, 
	-0.701548f, 0.712622f, 0.692884f, 
	-0.685358f, 0.728206f, 0.691635f, 
	-0.668822f, 0.743422f, 0.690387f, 
	-0.651949f, 0.758263f, 0.689139f, 
	-0.634746f, 0.772721f, 0.687890f, 
	-0.617223f, 0.786789f, 0.686642f, 
	-0.599388f, 0.800459f, 0.685393f, 
	-0.581250f, 0.813725f, 0.684145f, 
	-0.562819f, 0.826580f, 0.682896f, 
	-0.544103f, 0.839018f, 0.681648f, 
	-0.525113f, 0.851032f, 0.680400f, 
	-0.505858f, 0.862617f, 0.679151f, 
	-0.486347f, 0.873766f, 0.677903f, 
	-0.466591f, 0.884473f, 0.676654f, 
	-0.446599f, 0.894734f, 0.675406f, 
	-0.426382f, 0.904543f, 0.674157f, 
	-0.405949f, 0.913896f, 0.672909f, 
	-0.385311f, 0.922787f, 0.671660f, 
	-0.364479f, 0.931212f, 0.670412f, 
	-0.343462f, 0.939166f, 0.669164f, 
	-0.322273f, 0.946647f, 0.667915f, 
	-0.300920f, 0.953649f, 0.666667f, 
	-0.279415f, 0.960170f, 0.665418f, 
	-0.257770f, 0.966206f, 0.664170f, 
	-0.235994f, 0.971754f, 0.662921f, 
	-0.214099f, 0.976812f, 0.661673f, 
	-0.192096f, 0.981376f, 0.660424f, 
	-0.169996f, 0.985445f, 0.659176f, 
	-0.147810f, 0.989016f, 0.657928f, 
	-0.125550f, 0.992087f, 0.656679f, 
	-0.103226f, 0.994658f, 0.655431f, 
	-0.080850f, 0.996726f, 0.654182f, 
	-0.058433f, 0.998291f, 0.652934f, 
	-0.035987f, 0.999352f, 0.651685f, 
	-0.013522f, 0.999909f, 0.650437f, 
	0.008949f, 0.999960f, 0.649189f, 
	0.031416f, 0.999506f, 0.647940f, 
	0.053867f, 0.998548f, 0.646692f, 
	0.076291f, 0.997086f, 0.645443f, 
	0.098676f, 0.995120f, 0.644195f, 
	0.121012f, 0.992651f, 0.642946f, 
	0.143286f, 0.989681f, 0.641698f, 
	0.165488f, 0.986212f, 0.640449f, 
	0.187606f, 0.982244f, 0.639201f, 
	0.209630f, 0.977781f, 0.637953f, 
	0.231548f, 0.972823f, 0.636704f, 
	0.253349f, 0.967375f, 0.635456f, 
	0.275022f, 0.961438f, 0.634207f, 
	0.296556f, 0.955015f, 0.632959f, 
	0.317940f, 0.948111f, 0.631710f, 
	0.339164f, 0.940727f, 0.630462f, 
	0.360217f, 0.932869f, 0.629213f, 
	0.381087f, 0.924539f, 0.627965f, 
	0.401765f, 0.915743f, 0.626717f, 
	0.422241f, 0.906484f, 0.625468f, 
	0.442503f, 0.896767f, 0.624220f, 
	0.462541f, 0.886598f, 0.622971f, 
	0.482347f, 0.875980f, 0.621723f, 
	0.501908f, 0.864921f, 0.620474f, 
	0.521216f, 0.853425f, 0.619226f, 
	0.540261f, 0.841497f, 0.617978f, 
	0.559033f, 0.829145f, 0.616729f, 
	0.577523f, 0.816375f, 0.615481f, 
	0.595721f, 0.803191f, 0.614232f, 
	0.613618f, 0.789603f, 0.612984f, 
	0.631206f, 0.775615f, 0.611735f, 
	0.648474f, 0.761236f, 0.610487f, 
	0.665416f, 0.746473f, 0.609238f, 
	0.682021f, 0.731333f, 0.607990f, 
	0.698282f, 0.715823f, 0.606742f, 
	0.714190f, 0.699952f, 0.605493f, 
	0.729738f, 0.683727f, 0.604245f, 
	0.744917f, 0.667157f, 0.602996f, 
	0.759720f, 0.650251f, 0.601748f, 
	0.774139f, 0.633015f, 0.600499f, 
	0.788168f, 0.615461f, 0.599251f, 
	0.801798f, 0.597595f, 0.598002f, 
	0.815023f, 0.579428f, 0.596754f, 
	0.827837f, 0.560968f, 0.595506f, 
	0.840233f, 0.542225f, 0.594257f, 
	0.852205f, 0.523208f, 0.593009f, 
	0.863746f, 0.503927f, 0.591760f, 
	0.874852f, 0.484391f, 0.590512f, 
	0.885515f, 0.464611f, 0.589263f, 
	0.895731f, 0.444596f, 0.588015f, 
	0.905495f, 0.424357f, 0.586767f, 
	0.914802f, 0.403903f, 0.585518f, 
	0.923647f, 0.383245f, 0.584270f, 
	0.932025f, 0.362394f, 0.583021f, 
	0.939933f, 0.341360f, 0.581773f, 
	0.947366f, 0.320154f, 0.580524f, 
	0.954320f, 0.298786f, 0.579276f, 
	0.960793f, 0.277267f, 0.578027f, 
	0.966781f, 0.255607f, 0.576779f, 
	0.972280f, 0.233819f, 0.575531f, 
	0.977288f, 0.211913f, 0.574282f, 
	0.981803f, 0.189900f, 0.573034f, 
	0.985823f, 0.167791f, 0.571785f, 
	0.989344f, 0.145597f, 0.570537f, 
	0.992366f, 0.123330f, 0.569288f, 
	0.994886f, 0.101000f, 0.568040f, 
	0.996905f, 0.078620f, 0.566792f, 
	0.998420f, 0.056199f, 0.565543f, 
	0.999430f, 0.033751f, 0.564295f, 
	0.999936f, 0.011285f, 0.563046f, 
	0.999937f, -0.011187f, 0.561798f, 
	0.999434f, -0.033652f, 0.560549f, 
	0.998425f, -0.056101f, 0.559301f, 
	0.996912f, -0.078522f, 0.558052f, 
	0.994896f, -0.100903f, 0.556804f, 
	0.992378f, -0.123232f, 0.555556f, 
	0.989358f, -0.145500f, 0.554307f, 
	0.985839f, -0.167694f, 0.553059f, 
	0.981822f, -0.189804f, 0.551810f, 
	0.977309f, -0.211817f, 0.550562f, 
	0.972303f, -0.233724f, 0.549313f, 
	0.966806f, -0.255513f, 0.548065f, 
	0.960820f, -0.277172f, 0.546816f, 
	0.954350f, -0.298692f, 0.545568f, 
	0.947397f, -0.320061f, 0.544320f, 
	0.939966f, -0.341268f, 0.543071f, 
	0.932060f, -0.362303f, 0.541823f, 
	0.923684f, -0.383155f, 0.540574f, 
	0.914841f, -0.403813f, 0.539326f, 
	0.905537f, -0.424268f, 0.538077f, 
	0.895775f, -0.444508f, 0.536829f, 
	0.885561f, -0.464524f, 0.535581f, 
	0.874899f, -0.484305f, 0.534332f, 
	0.863796f, -0.503842f, 0.533084f, 
	0.852256f, -0.523124f, 0.531835f, 
	0.840287f, -0.542142f, 0.530587f, 
	0.827893f, -0.560887f, 0.529338f, 
	0.815080f, -0.579348f, 0.528090f, 
	0.801857f, -0.597517f, 0.526841f, 
	0.788228f, -0.615383f, 0.525593f, 
	0.774201f, -0.632940f, 0.524345f, 
	0.759784f, -0.650176f, 0.523096f, 
	0.744982f, -0.667084f, 0.521848f, 
	0.729805f, -0.683656f, 0.520599f, 
	0.714259f, -0.699882f, 0.519351f, 
	0.698352f, -0.715754f, 0.518102f, 
	0.682093f, -0.731266f, 0.516854f, 
	0.665489f, -0.746408f, 0.515605f, 
	0.648549f, -0.761173f, 0.514357f, 
	0.631282f, -0.775553f, 0.513109f, 
	0.613696f, -0.789543f, 0.511860f, 
	0.595800f, -0.803133f, 0.510612f, 
	0.577603f, -0.816318f, 0.509363f, 
	0.559114f, -0.829091f, 0.508115f, 
	0.540344f, -0.841444f, 0.506866f, 
	0.521300f, -0.853374f, 0.505618f, 
	0.501993f, -0.864872f, 0.504370f, 
	0.482433f, -0.875933f, 0.503121f, 
	0.462628f, -0.886552f, 0.501873f, 
	0.442591f, -0.896724f, 0.500624f, 
	0.422330f, -0.906442f, 0.499376f, 
	0.401855f, -0.915703f, 0.498127f, 
	0.381178f, -0.924502f, 0.496879f, 
	0.360308f, -0.932833f, 0.495630f, 
	0.339256f, -0.940694f, 0.494382f, 
	0.318033f, -0.948080f, 0.493134f, 
	0.296650f, -0.954986f, 0.491885f, 
	0.275116f, -0.961411f, 0.490637f, 
	0.253444f, -0.967350f, 0.489388f, 
	0.231643f, -0.972801f, 0.488140f, 
	0.209726f, -0.977760f, 0.486891f, 
	0.187703f, -0.982226f, 0.485643f, 
	0.165585f, -0.986196f, 0.484395f, 
	0.143383f, -0.989667f, 0.483146f, 
	0.121109f, -0.992639f, 0.481898f, 
	0.098774f, -0.995110f, 0.480649f, 
	0.076389f, -0.997078f, 0.479401f, 
	0.053965f, -0.998543f, 0.478152f, 
	0.031514f, -0.999503f, 0.476904f, 
	0.009048f, -0.999959f, 0.475655f, 
	-0.013424f, -0.999910f, 0.474407f, 
	-0.035888f, -0.999356f, 0.473159f, 
	-0.058335f, -0.998297f, 0.471910f, 
	-0.080752f, -0.996734f, 0.470662f, 
	-0.103128f, -0.994668f, 0.469413f, 
	-0.125452f, -0.992100f, 0.468165f, 
	-0.147713f, -0.989030f, 0.466916f, 
	-0.169899f, -0.985461f, 0.465668f, 
	-0.192000f, -0.981395f, 0.464419f, 
	-0.214003f, -0.976833f, 0.463171f, 
	-0.235899f, -0.971778f, 0.461923f, 
	-0.257675f, -0.966232f, 0.460674f, 
	-0.279321f, -0.960198f, 0.459426f, 
	-0.300826f, -0.953679f, 0.458177f, 
	-0.322180f, -0.946679f, 0.456929f, 
	-0.343370f, -0.939200f, 0.455680f, 
	-0.364387f, -0.931247f, 0.454432f, 
	-0.385221f, -0.922825f, 0.453184f, 
	-0.405859f, -0.913936f, 0.451935f, 
	-0.426293f, -0.904585f, 0.450687f, 
	-0.446511f, -0.894778f, 0.449438f, 
	-0.466504f, -0.884519f, 0.448190f, 
	-0.486262f, -0.873813f, 0.446941f, 
	-0.505773f, -0.862666f, 0.445693f, 
	-0.525030f, -0.851084f, 0.444444f, 
	-0.544021f, -0.839072f, 0.443196f, 
	-0.562738f, -0.826636f, 0.441948f, 
	-0.581170f, -0.813782f, 0.440699f, 
	-0.599309f, -0.800518f, 0.439451f, 
	-0.617145f, -0.786849f, 0.438202f, 
	-0.634670f, -0.772783f, 0.436954f, 
	-0.651874f, -0.758327f, 0.435705f, 
	-0.668749f, -0.743488f, 0.434457f, 
	-0.685287f, -0.728273f, 0.433208f, 
	-0.701478f, -0.712691f, 0.431960f, 
	-0.717315f, -0.696749f, 0.430712f, 
	-0.732790f, -0.680455f, 0.429463f, 
	-0.747895f, -0.663817f, 0.428215f, 
	-0.762622f, -0.646845f, 0.426966f, 
	-0.776964f, -0.629545f, 0.425718f, 
	-0.790914f, -0.611928f, 0.424469f, 
	-0.804464f, -0.594001f, 0.423221f, 
	-0.817608f, -0.575775f, 0.421973f, 
	-0.830339f, -0.557258f, 0.420724f, 
	-0.842651f, -0.538460f, 0.419476f, 
	-0.854538f, -0.519389f, 0.418227f, 
	-0.865993f, -0.500057f, 0.416979f, 
	-0.877010f, -0.480472f, 0.415730f, 
	-0.887585f, -0.460644f, 0.414482f, 
	-0.897712f, -0.440583f, 0.413233f, 
	-0.907385f, -0.420301f, 0.411985f, 
	-0.916600f, -0.399805f, 0.410737f, 
	-0.925352f, -0.379109f, 0.409488f, 
	-0.933637f, -0.358220f, 0.408240f, 
	-0.941451f, -0.337151f, 0.406991f, 
	-0.948789f, -0.315911f, 0.405743f, 
	-0.955648f, -0.294512f, 0.404494f, 
	-0.962024f, -0.272964f, 0.403246f, 
	-0.967915f, -0.251279f, 0.401998f, 
	-0.973317f, -0.229466f, 0.400749f, 
	-0.978227f, -0.207538f, 0.399501f, 
	-0.982643f, -0.185505f, 0.398252f, 
	-0.986564f, -0.163378f, 0.397004f, 
	-0.989986f, -0.141169f, 0.395755f, 
	-0.992908f, -0.118888f, 0.394507f, 
	-0.995328f, -0.096547f, 0.393258f, 
	-0.997247f, -0.074158f, 0.392010f, 
	-0.998661f, -0.051731f, 0.390762f, 
	-0.999571f, -0.029278f, 0.389513f, 
	-0.999977f, -0.006810f, 0.388265f, 
	-0.999877f, 0.015661f, 0.387016f, 
	-0.999273f, 0.038124f, 0.385768f, 
	-0.998164f, 0.060568f, 0.384519f, 
	-0.996551f, 0.082982f, 0.383271f, 
	-0.994435f, 0.105353f, 0.382022f, 
	-0.991816f, 0.127672f, 0.380774f, 
	-0.988697f, 0.149926f, 0.379526f, 
	-0.985079f, 0.172104f, 0.378277f, 
	-0.980963f, 0.194195f, 0.377029f, 
	-0.976352f, 0.216188f, 0.375780f, 
	-0.971247f, 0.238072f, 0.374532f, 
	-0.965653f, 0.259836f, 0.373283f, 
	-0.959570f, 0.281469f, 0.372035f, 
	-0.953003f, 0.302959f, 0.370787f, 
	-0.945955f, 0.324297f, 0.369538f, 
	-0.938430f, 0.345471f, 0.368290f, 
	-0.930430f, 0.366470f, 0.367041f, 
	-0.921960f, 0.387284f, 0.365793f, 
	-0.913025f, 0.407903f, 0.364544f, 
	-0.903629f, 0.428316f, 0.363296f, 
	-0.893777f, 0.448512f, 0.362047f, 
	-0.883473f, 0.468482f, 0.360799f, 
	-0.872723f, 0.488215f, 0.359551f, 
	-0.861533f, 0.507702f, 0.358302f, 
	-0.849907f, 0.526933f, 0.357054f, 
	-0.837852f, 0.545897f, 0.355805f, 
	-0.825374f, 0.564586f, 0.354557f, 
	-0.812480f, 0.582989f, 0.353308f, 
	-0.799175f, 0.601099f, 0.352060f, 
	-0.785466f, 0.618904f, 0.350811f, 
	-0.771361f, 0.636398f, 0.349563f, 
	-0.756867f, 0.653569f, 0.348315f, 
	-0.741990f, 0.670411f, 0.347066f, 
	-0.726738f, 0.686914f, 0.345818f, 
	-0.711120f, 0.703071f, 0.344569f, 
	-0.695142f, 0.718872f, 0.343321f, 
	-0.678814f, 0.734311f, 0.342072f, 
	-0.662142f, 0.749378f, 0.340824f, 
	-0.645137f, 0.764067f, 0.339576f, 
	-0.627805f, 0.778371f, 0.338327f, 
	-0.610157f, 0.792281f, 0.337079f, 
	-0.592200f, 0.805791f, 0.335830f, 
	-0.573944f, 0.818894f, 0.334582f, 
	-0.555399f, 0.831584f, 0.333333f, 
	-0.536573f, 0.843854f, 0.332085f, 
	-0.517476f, 0.855698f, 0.330836f, 
	-0.498118f, 0.867109f, 0.329588f, 
	-0.478508f, 0.878083f, 0.328340f, 
	-0.458657f, 0.888614f, 0.327091f, 
	-0.438574f, 0.898695f, 0.325843f, 
	-0.418269f, 0.908323f, 0.324594f, 
	-0.397754f, 0.917492f, 0.323346f, 
	-0.377037f, 0.926198f, 0.322097f, 
	-0.356130f, 0.934436f, 0.320849f, 
	-0.335044f, 0.942203f, 0.319600f, 
	-0.313788f, 0.949493f, 0.318352f, 
	-0.292373f, 0.956304f, 0.317104f, 
	-0.270811f, 0.962632f, 0.315855f, 
	-0.249113f, 0.968475f, 0.314607f, 
	-0.227288f, 0.973828f, 0.313358f, 
	-0.205349f, 0.978689f, 0.312110f, 
	-0.183306f, 0.983056f, 0.310861f, 
	-0.161170f, 0.986927f, 0.309613f, 
	-0.138953f, 0.990299f, 0.308365f, 
	-0.116666f, 0.993171f, 0.307116f, 
	-0.094320f, 0.995542f, 0.305868f, 
	-0.071926f, 0.997410f, 0.304619f, 
	-0.049496f, 0.998774f, 0.303371f, 
	-0.027041f, 0.999634f, 0.302122f, 
	-0.004573f, 0.999990f, 0.300874f, 
	0.017898f, 0.999840f, 0.299625f, 
	0.040360f, 0.999185f, 0.298377f, 
	0.062802f, 0.998026f, 0.297129f, 
	0.085211f, 0.996363f, 0.295880f, 
	0.107578f, 0.994197f, 0.294632f, 
	0.129891f, 0.991528f, 0.293383f, 
	0.152137f, 0.988359f, 0.292135f, 
	0.174307f, 0.984691f, 0.290886f, 
	0.196389f, 0.980526f, 0.289638f, 
	0.218372f, 0.975866f, 0.288390f, 
	0.240245f, 0.970712f, 0.287141f, 
	0.261996f, 0.965069f, 0.285893f, 
	0.283615f, 0.958938f, 0.284644f, 
	0.305091f, 0.952323f, 0.283396f, 
	0.326413f, 0.945227f, 0.282147f, 
	0.347569f, 0.937654f, 0.280899f, 
	0.368551f, 0.929608f, 0.279650f, 
	0.389346f, 0.921092f, 0.278402f, 
	0.409945f, 0.912110f, 0.277154f, 
	0.430336f, 0.902669f, 0.275905f, 
	0.450511f, 0.892771f, 0.274657f, 
	0.470457f, 0.882423f, 0.273408f, 
	0.490167f, 0.871629f, 0.272160f, 
	0.509629f, 0.860395f, 0.270911f, 
	0.528833f, 0.848726f, 0.269663f, 
	0.547770f, 0.836629f, 0.268414f, 
	0.566431f, 0.824109f, 0.267166f, 
	0.584806f, 0.811173f, 0.265918f, 
	0.602885f, 0.797828f, 0.264669f, 
	0.620660f, 0.784080f, 0.263421f, 
	0.638122f, 0.769935f, 0.262172f, 
	0.655261f, 0.755402f, 0.260924f, 
	0.672070f, 0.740488f, 0.259675f, 
	0.688539f, 0.725200f, 0.258427f, 
	0.704660f, 0.709545f, 0.257179f, 
	0.720426f, 0.693532f, 0.255930f, 
	0.735827f, 0.677169f, 0.254682f, 
	0.750858f, 0.660464f, 0.253433f, 
	0.765509f, 0.643425f, 0.252185f, 
	0.779773f, 0.626062f, 0.250936f, 
	0.793644f, 0.608382f, 0.249688f, 
	0.807114f, 0.590396f, 0.248439f, 
	0.820176f, 0.572111f, 0.247191f, 
	0.832825f, 0.553537f, 0.245943f, 
	0.845052f, 0.534684f, 0.244694f, 
	0.856853f, 0.515560f, 0.243446f, 
	0.868222f, 0.496177f, 0.242197f, 
	0.879152f, 0.476542f, 0.240949f, 
	0.889637f, 0.456667f, 0.239700f, 
	0.899674f, 0.436562f, 0.238452f, 
	0.909257f, 0.416236f, 0.237203f, 
	0.918380f, 0.395700f, 0.235955f, 
	0.927039f, 0.374964f, 0.234707f, 
	0.935231f, 0.354039f, 0.233458f, 
	0.942950f, 0.332935f, 0.232210f, 
	0.950193f, 0.311663f, 0.230961f, 
	0.956956f, 0.290233f, 0.229713f, 
	0.963236f, 0.268657f, 0.228464f, 
	0.969029f, 0.246945f, 0.227216f, 
	0.974334f, 0.225109f, 0.225968f, 
	0.979146f, 0.203159f, 0.224719f, 
	0.983464f, 0.181106f, 0.223471f, 
	0.987285f, 0.158962f, 0.222222f, 
	0.990607f, 0.136737f, 0.220974f, 
	0.993430f, 0.114444f, 0.219725f, 
	0.995750f, 0.092092f, 0.218477f, 
	0.997568f, 0.069695f, 0.217228f, 
	0.998883f, 0.047262f, 0.215980f, 
	0.999692f, 0.024805f, 0.214732f, 
	0.999997f, 0.002335f, 0.213483f, 
	0.999797f, -0.020135f, 0.212235f, 
	0.999092f, -0.042595f, 0.210986f, 
	0.997883f, -0.065034f, 0.209738f, 
	0.996170f, -0.087440f, 0.208489f, 
	0.993953f, -0.109802f, 0.207241f, 
	0.991235f, -0.132109f, 0.205993f, 
	0.988016f, -0.154348f, 0.204744f, 
	0.984299f, -0.176510f, 0.203496f, 
	0.980084f, -0.198583f, 0.202247f, 
	0.975375f, -0.220555f, 0.200999f, 
	0.970172f, -0.242416f, 0.199750f, 
	0.964480f, -0.264155f, 0.198502f, 
	0.958301f, -0.285760f, 0.197253f, 
	0.951638f, -0.307221f, 0.196005f, 
	0.944495f, -0.328527f, 0.194757f, 
	0.936874f, -0.349666f, 0.193508f, 
	0.928781f, -0.370630f, 0.192260f, 
	0.920218f, -0.391406f, 0.191011f, 
	0.911191f, -0.411984f, 0.189763f, 
	0.901704f, -0.432355f, 0.188514f, 
	0.891761f, -0.452507f, 0.187266f, 
	0.881368f, -0.472431f, 0.186017f, 
	0.870530f, -0.492116f, 0.184769f, 
	0.859252f, -0.511552f, 0.183521f, 
	0.847541f, -0.530731f, 0.182272f, 
	0.835401f, -0.549641f, 0.181024f, 
	0.822840f, -0.568273f, 0.179775f, 
	0.809863f, -0.586619f, 0.178527f, 
	0.796477f, -0.604669f, 0.177278f, 
	0.782689f, -0.622413f, 0.176030f, 
	0.768506f, -0.639843f, 0.174782f, 
	0.753934f, -0.656950f, 0.173533f, 
	0.738982f, -0.673725f, 0.172285f, 
	0.723657f, -0.690160f, 0.171036f, 
	0.707967f, -0.706246f, 0.169788f, 
	0.691919f, -0.721976f, 0.168539f, 
	0.675521f, -0.737341f, 0.167291f, 
	0.658782f, -0.752334f, 0.166042f, 
	0.641711f, -0.766946f, 0.164794f, 
	0.624316f, -0.781172f, 0.163546f, 
	0.606605f, -0.795003f, 0.162297f, 
	0.588588f, -0.808433f, 0.161049f, 
	0.570274f, -0.821454f, 0.159800f, 
	0.551672f, -0.834061f, 0.158552f, 
	0.532792f, -0.846247f, 0.157303f, 
	0.513642f, -0.858005f, 0.156055f, 
	0.494233f, -0.869330f, 0.154806f, 
	0.474574f, -0.880216f, 0.153558f, 
	0.454676f, -0.890657f, 0.152310f, 
	0.434548f, -0.900649f, 0.151061f, 
	0.414201f, -0.910186f, 0.149813f, 
	0.393644f, -0.919263f, 0.148564f, 
	0.372889f, -0.927876f, 0.147316f, 
	0.351945f, -0.936021f, 0.146067f, 
	0.330824f, -0.943692f, 0.144819f, 
	0.309536f, -0.950888f, 0.143571f, 
	0.288091f, -0.957603f, 0.142322f, 
	0.266501f, -0.963835f, 0.141074f, 
	0.244776f, -0.969580f, 0.139825f, 
	0.222928f, -0.974835f, 0.138577f, 
	0.200967f, -0.979598f, 0.137328f, 
	0.178905f, -0.983866f, 0.136080f, 
	0.156752f, -0.987638f, 0.134831f, 
	0.134521f, -0.990911f, 0.133583f, 
	0.112221f, -0.993683f, 0.132335f, 
	0.089864f, -0.995954f, 0.131086f, 
	0.067463f, -0.997722f, 0.129838f, 
	0.045027f, -0.998986f, 0.128589f, 
	0.022568f, -0.999745f, 0.127341f, 
	0.000098f, -1.000000f, 0.126092f, 
	-0.022372f, -0.999750f, 0.124844f, 
	-0.044831f, -0.998995f, 0.123596f, 
	-0.067267f, -0.997735f, 0.122347f, 
	-0.089669f, -0.995972f, 0.121099f, 
	-0.112026f, -0.993705f, 0.119850f, 
	-0.134326f, -0.990937f, 0.118602f, 
	-0.156559f, -0.987669f, 0.117353f, 
	-0.178712f, -0.983901f, 0.116105f, 
	-0.200775f, -0.979637f, 0.114856f, 
	-0.222737f, -0.974879f, 0.113608f, 
	-0.244586f, -0.969628f, 0.112360f, 
	-0.266312f, -0.963887f, 0.111111f, 
	-0.287903f, -0.957659f, 0.109863f, 
	-0.309349f, -0.950948f, 0.108614f, 
	-0.330639f, -0.943757f, 0.107366f, 
	-0.351762f, -0.936090f, 0.106117f, 
	-0.372707f, -0.927949f, 0.104869f, 
	-0.393464f, -0.919340f, 0.103620f, 
	-0.414022f, -0.910267f, 0.102372f, 
	-0.434371f, -0.900734f, 0.101124f, 
	-0.454501f, -0.890746f, 0.099875f, 
	-0.474401f, -0.880309f, 0.098627f, 
	-0.494062f, -0.869427f, 0.097378f, 
	-0.513473f, -0.858105f, 0.096130f, 
	-0.532625f, -0.846351f, 0.094881f, 
	-0.551509f, -0.834169f, 0.093633f, 
	-0.570113f, -0.821566f, 0.092385f, 
	-0.588430f, -0.808548f, 0.091136f, 
	-0.606449f, -0.795122f, 0.089888f, 
	-0.624163f, -0.781295f, 0.088639f, 
	-0.641561f, -0.767072f, 0.087391f, 
	-0.658635f, -0.752463f, 0.086142f, 
	-0.675376f, -0.737473f, 0.084894f, 
	-0.691777f, -0.722111f, 0.083645f, 
	-0.707828f, -0.706385f, 0.082397f, 
	-0.723522f, -0.690301f, 0.081149f, 
	-0.738850f, -0.673870f, 0.079900f, 
	-0.753806f, -0.657098f, 0.078652f, 
	-0.768380f, -0.639994f, 0.077403f, 
	-0.782567f, -0.622566f, 0.076155f, 
	-0.796358f, -0.604825f, 0.074906f, 
	-0.809748f, -0.586778f, 0.073658f, 
	-0.822728f, -0.568435f, 0.072409f, 
	-0.835293f, -0.549805f, 0.071161f, 
	-0.847436f, -0.530897f, 0.069913f, 
	-0.859152f, -0.511721f, 0.068664f, 
	-0.870433f, -0.492286f, 0.067416f, 
	-0.881275f, -0.472604f, 0.066167f, 
	-0.891672f, -0.452682f, 0.064919f, 
	-0.901619f, -0.432532f, 0.063670f, 
	-0.911110f, -0.412163f, 0.062422f, 
	-0.920141f, -0.391586f, 0.061174f, 
	-0.928708f, -0.370812f, 0.059925f, 
	-0.936806f, -0.349850f, 0.058677f, 
	-0.944430f, -0.328712f, 0.057428f, 
	-0.951578f, -0.307408f, 0.056180f, 
	-0.958245f, -0.285948f, 0.054931f, 
	-0.964428f, -0.264344f, 0.053683f, 
	-0.970125f, -0.242607f, 0.052434f, 
	-0.975331f, -0.220747f, 0.051186f, 
	-0.980045f, -0.198775f, 0.049938f, 
	-0.984264f, -0.176703f, 0.048689f, 
	-0.987986f, -0.154542f, 0.047441f, 
	-0.991209f, -0.132303f, 0.046192f, 
	-0.993932f, -0.109997f, 0.044944f, 
	-0.996153f, -0.087636f, 0.043695f, 
	-0.997870f, -0.065230f, 0.042447f, 
	-0.999084f, -0.042791f, 0.041199f, 
	-0.999793f, -0.020331f, 0.039950f, 
	-0.999998f, 0.002139f, 0.038702f, 
	-0.999697f, 0.024609f, 0.037453f, 
	-0.998892f, 0.047066f, 0.036205f, 
	-0.997582f, 0.069499f, 0.034956f, 
	-0.995769f, 0.091897f, 0.033708f, 
	-0.993452f, 0.114249f, 0.032459f, 
	-0.990634f, 0.136543f, 0.031211f, 
	-0.987316f, 0.158768f, 0.029963f, 
	-0.983499f, 0.180913f, 0.028714f, 
	-0.979186f, 0.202966f, 0.027466f, 
	-0.974378f, 0.224918f, 0.026217f, 
	-0.969078f, 0.246755f, 0.024969f, 
	-0.963289f, 0.268468f, 0.023720f, 
	-0.957013f, 0.290045f, 0.022472f, 
	-0.950254f, 0.311476f, 0.021223f, 
	-0.943015f, 0.332750f, 0.019975f, 
	-0.935300f, 0.353855f, 0.018727f, 
	-0.927113f, 0.374782f, 0.017478f, 
	-0.918457f, 0.395520f, 0.016230f, 
	-0.909338f, 0.416058f, 0.014981f, 
	-0.899760f, 0.436385f, 0.013733f, 
	-0.889727f, 0.456493f, 0.012484f, 
	-0.879245f, 0.476370f, 0.011236f, 
	-0.868319f, 0.496006f, 0.009988f, 
	-0.856954f, 0.515392f, 0.008739f, 
	-0.845157f, 0.534518f, 0.007491f, 
	-0.832933f, 0.553373f, 0.006242f, 
	-0.820289f, 0.571950f, 0.004994f, 
	-0.807230f, 0.590237f, 0.003745f, 
	-0.793763f, 0.608227f, 0.002497f, 
	-0.779896f, 0.625909f, 0.001248f, 
	-0.765635f, 0.643275f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f, 
	0.000000f, 0.000000f, 0.000000f
};

