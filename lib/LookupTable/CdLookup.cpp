#include "CdLookup.h"
#include "RocketConfig.h"
#include <math.h>

// CFD lookup table from LookupTableV1.csv
// 25 airbrake angles (1° to 95°, uniform spacing 3.9167°)
// 25 Mach numbers (0.1 to 0.8, uniform spacing 0.02917)
// CD_TABLE[mach_idx][angle_idx] = total coefficient of drag

static constexpr int ANGLE_COUNT = 25;
static constexpr int MACH_COUNT = 25;
static constexpr float ANGLE_MIN = 1.0f;
static constexpr float ANGLE_MAX = 95.0f;
static constexpr float ANGLE_STEP = (ANGLE_MAX - ANGLE_MIN) / (ANGLE_COUNT - 1);  // 3.9167
static constexpr float MACH_MIN = 0.1f;
static constexpr float MACH_MAX = 0.8f;
static constexpr float MACH_STEP = (MACH_MAX - MACH_MIN) / (MACH_COUNT - 1);  // 0.02917

static constexpr float CD_TABLE[MACH_COUNT][ANGLE_COUNT] = {
    {0.582064f, 0.625153f, 0.669628f, 0.715518f, 0.762796f, 0.811378f, 0.861129f, 0.911880f, 0.963454f, 1.015701f, 1.068531f, 1.121957f, 1.176149f, 1.231541f, 1.290504f, 1.352705f, 1.416671f, 1.482283f, 1.549573f, 1.618584f, 1.689323f, 1.761740f, 1.835732f, 1.911146f, 1.987804f},
    {0.574490f, 0.617772f, 0.662463f, 0.708583f, 0.756095f, 0.804900f, 0.854847f, 0.905751f, 0.957426f, 1.009714f, 1.062526f, 1.115882f, 1.169961f, 1.225203f, 1.283944f, 1.346037f, 1.409955f, 1.475587f, 1.542981f, 1.612189f, 1.683223f, 1.756032f, 1.830503f, 1.906468f, 1.983729f},
    {0.567438f, 0.610926f, 0.655834f, 0.702174f, 0.749896f, 0.798888f, 0.848986f, 0.899997f, 0.951730f, 1.004027f, 1.056807f, 1.110098f, 1.164089f, 1.219223f, 1.277796f, 1.339834f, 1.403750f, 1.469435f, 1.536945f, 1.606344f, 1.677648f, 1.750811f, 1.825714f, 1.902182f, 1.979997f},
    {0.560983f, 0.604701f, 0.649838f, 0.696396f, 0.744313f, 0.793466f, 0.843683f, 0.894770f, 0.946537f, 0.998837f, 1.051599f, 1.104862f, 1.158825f, 1.213929f, 1.272418f, 1.334478f, 1.398455f, 1.464232f, 1.531868f, 1.601435f, 1.672961f, 1.746406f, 1.821657f, 1.898537f, 1.976821f},
    {0.555187f, 0.599166f, 0.644553f, 0.691338f, 0.739447f, 0.788749f, 0.839070f, 0.890219f, 0.942020f, 0.994339f, 1.047123f, 1.100425f, 1.154449f, 1.209627f, 1.268139f, 1.330318f, 1.394430f, 1.460340f, 1.528106f, 1.597807f, 1.669483f, 1.743113f, 1.818596f, 1.895765f, 1.974395f},
    {0.550089f, 0.594367f, 0.640035f, 0.687065f, 0.735374f, 0.784825f, 0.835247f, 0.886462f, 0.938311f, 0.990684f, 1.043549f, 1.096973f, 1.151162f, 1.206533f, 1.265188f, 1.327590f, 1.391916f, 1.458003f, 1.525900f, 1.595692f, 1.667437f, 1.741138f, 1.816720f, 1.894034f, 1.972866f},
    {0.545696f, 0.590318f, 0.636299f, 0.683599f, 0.732123f, 0.781731f, 0.832261f, 0.883552f, 0.935472f, 0.987940f, 1.040948f, 1.094578f, 1.149036f, 1.204713f, 1.263628f, 1.326352f, 1.390968f, 1.457274f, 1.525304f, 1.595148f, 1.666884f, 1.740547f, 1.816095f, 1.893411f, 1.972301f},
    {0.541983f, 0.586988f, 0.633317f, 0.680911f, 0.729665f, 0.779440f, 0.830084f, 0.881461f, 0.933470f, 0.986063f, 1.039262f, 1.093164f, 1.147971f, 1.204047f, 1.263315f, 1.326446f, 1.391419f, 1.457985f, 1.526157f, 1.596027f, 1.667699f, 1.741239f, 1.816648f, 1.893847f, 1.972670f},
    {0.538887f, 0.584308f, 0.631008f, 0.678914f, 0.727907f, 0.777851f, 0.828608f, 0.880068f, 0.932165f, 0.984890f, 1.038297f, 1.092500f, 1.147699f, 1.204225f, 1.263905f, 1.327500f, 1.392880f, 1.459744f, 1.528081f, 1.597982f, 1.669570f, 1.742948f, 1.818161f, 1.895169f, 1.973844f},
    {0.536314f, 0.582168f, 0.629252f, 0.677471f, 0.726700f, 0.776800f, 0.827651f, 0.879169f, 0.931328f, 0.984159f, 1.037750f, 1.092237f, 1.147817f, 1.204794f, 1.264901f, 1.328980f, 1.394799f, 1.462000f, 1.530543f, 1.600513f, 1.672047f, 1.745283f, 1.820305f, 1.897113f, 1.975615f},
    {0.534146f, 0.580431f, 0.627890f, 0.676410f, 0.725851f, 0.776078f, 0.826982f, 0.878511f, 0.930676f, 0.983551f, 1.037259f, 1.091962f, 1.147861f, 1.205237f, 1.265738f, 1.330288f, 1.396557f, 1.464131f, 1.532938f, 1.603050f, 1.674615f, 1.747791f, 1.822695f, 1.899363f, 1.977732f},
    {0.532255f, 0.578948f, 0.626755f, 0.675540f, 0.725154f, 0.775458f, 0.826360f, 0.877833f, 0.929924f, 0.982750f, 1.036474f, 1.091284f, 1.147394f, 1.205074f, 1.265895f, 1.330870f, 1.397580f, 1.465560f, 1.534703f, 1.605065f, 1.676787f, 1.750039f, 1.824959f, 1.901607f, 1.979942f},
    {0.530517f, 0.577575f, 0.625680f, 0.674681f, 0.724412f, 0.774735f, 0.825568f, 0.876907f, 0.928835f, 0.981507f, 1.035125f, 1.089912f, 1.146098f, 1.203957f, 1.264998f, 1.330327f, 1.397456f, 1.465869f, 1.535427f, 1.606159f, 1.678194f, 1.751696f, 1.826806f, 1.903596f, 1.982040f},
    {0.528822f, 0.576185f, 0.624529f, 0.673682f, 0.723470f, 0.773751f, 0.824449f, 0.875583f, 0.927263f, 0.979681f, 1.033079f, 1.087711f, 1.143839f, 1.201744f, 1.262893f, 1.328497f, 1.396008f, 1.464872f, 1.534918f, 1.606143f, 1.678650f, 1.752583f, 1.828073f, 1.905186f, 1.983899f},
    {0.527080f, 0.574679f, 0.623193f, 0.672436f, 0.722223f, 0.772408f, 0.822923f, 0.873799f, 0.925173f, 0.977267f, 1.030356f, 1.084731f, 1.140683f, 1.198518f, 1.259674f, 1.325472f, 1.393324f, 1.462646f, 1.533236f, 1.605058f, 1.678178f, 1.752707f, 1.828750f, 1.906355f, 1.985489f},
    {0.525239f, 0.573002f, 0.621620f, 0.670896f, 0.720635f, 0.770691f, 0.820999f, 0.871601f, 0.922650f, 0.974392f, 1.027130f, 1.081188f, 1.136892f, 1.194576f, 1.255659f, 1.321584f, 1.389735f, 1.459507f, 1.530673f, 1.603161f, 1.676994f, 1.752243f, 1.828972f, 1.907200f, 1.986876f},
    {0.523286f, 0.571148f, 0.619813f, 0.669076f, 0.718742f, 0.768661f, 0.818772f, 0.869121f, 0.919872f, 0.971286f, 1.023688f, 1.077428f, 1.132864f, 1.190359f, 1.251326f, 1.317330f, 1.385741f, 1.455942f, 1.527685f, 1.600865f, 1.675461f, 1.751494f, 1.828983f, 1.907909f, 1.988198f},
    {0.521248f, 0.569153f, 0.617821f, 0.667046f, 0.716633f, 0.766434f, 0.816389f, 0.866548f, 0.917075f, 0.968237f, 1.020372f, 1.073849f, 1.129050f, 1.186371f, 1.247216f, 1.313275f, 1.381914f, 1.452512f, 1.524804f, 1.598656f, 1.674005f, 1.750823f, 1.829080f, 1.908718f, 1.989633f},
    {0.519179f, 0.567089f, 0.615732f, 0.664911f, 0.714434f, 0.764160f, 0.814030f, 0.864089f, 0.914501f, 0.965530f, 1.017514f, 1.070830f, 1.125880f, 1.183083f, 1.243836f, 1.309951f, 1.378797f, 1.449749f, 1.522538f, 1.597003f, 1.673046f, 1.750591f, 1.829564f, 1.909866f, 1.991366f},
    {0.517153f, 0.565042f, 0.613650f, 0.662791f, 0.712284f, 0.761994f, 0.811864f, 0.861937f, 0.912366f, 0.963403f, 1.015379f, 1.068670f, 1.123686f, 1.180861f, 1.241581f, 1.307775f, 1.376815f, 1.448080f, 1.521296f, 1.596289f, 1.672926f, 1.751095f, 1.830681f, 1.911552f, 1.993550f},
    {0.515247f, 0.563106f, 0.611684f, 0.660808f, 0.710313f, 0.760075f, 0.810038f, 0.860240f, 0.810821f, 0.962016f, 1.014138f, 1.067553f, 1.122667f, 1.179923f, 1.240688f, 1.306999f, 1.376232f, 1.447769f, 1.521340f, 1.596758f, 1.673869f, 1.752532f, 1.832599f, 1.913910f, 1.996290f},
    {0.513537f, 0.561369f, 0.609930f, 0.659066f, 0.708631f, 0.758510f, 0.808653f, 0.859091f, 0.909948f, 0.961439f, 1.013852f, 1.067533f, 1.122879f, 1.180328f, 1.241224f, 1.307697f, 1.377130f, 1.448907f, 1.522762f, 1.598503f, 1.675963f, 1.754980f, 1.835384f, 1.916996f, 1.999627f},
    {0.512083f, 0.559902f, 0.608467f, 0.657647f, 0.707314f, 0.757368f, 0.807763f, 0.858525f, 0.909761f, 0.961661f, 1.014486f, 1.068557f, 1.124251f, 1.181997f, 1.243103f, 1.309790f, 1.379437f, 1.451429f, 1.525507f, 1.601479f, 1.679172f, 1.758410f, 1.839012f, 1.920789f, 2.003544f},
    {0.510931f, 0.558755f, 0.607350f, 0.656604f, 0.706410f, 0.756683f, 0.807383f, 0.858530f, 0.910217f, 0.962608f, 1.015934f, 1.070487f, 1.126621f, 1.184747f, 1.246135f, 1.313079f, 1.382957f, 1.455150f, 1.529404f, 1.605531f, 1.683357f, 1.762703f, 1.843383f, 1.925204f, 2.007971f},
    {0.510109f, 0.557960f, 0.606608f, 0.655961f, 0.705934f, 0.756455f, 0.807491f, 0.859059f, 0.911237f, 0.964165f, 1.018046f, 1.073141f, 1.129776f, 1.188343f, 1.250063f, 1.317301f, 1.387425f, 1.459811f, 1.534208f, 1.610435f, 1.688318f, 1.767683f, 1.848344f, 1.930113f, 2.012803f}
};

// 2D bilinear interpolation on the CFD table
float CdLookup::lookupCd(float angle_deg, float mach) {
    // Clamp to table bounds
    if (angle_deg < ANGLE_MIN) angle_deg = ANGLE_MIN;
    if (angle_deg > ANGLE_MAX) angle_deg = ANGLE_MAX;
    if (mach < MACH_MIN) mach = MACH_MIN;
    if (mach > MACH_MAX) mach = MACH_MAX;

    // Compute fractional indices
    float ai = (angle_deg - ANGLE_MIN) / ANGLE_STEP;
    float mi = (mach - MACH_MIN) / MACH_STEP;

    int a0 = (int)ai;
    int m0 = (int)mi;
    if (a0 >= ANGLE_COUNT - 1) a0 = ANGLE_COUNT - 2;
    if (m0 >= MACH_COUNT - 1) m0 = MACH_COUNT - 2;

    float af = ai - a0;  // fractional part [0, 1]
    float mf = mi - m0;

    // Bilinear interpolation
    float c00 = CD_TABLE[m0][a0];
    float c10 = CD_TABLE[m0][a0 + 1];
    float c01 = CD_TABLE[m0 + 1][a0];
    float c11 = CD_TABLE[m0 + 1][a0 + 1];

    float c0 = c00 + af * (c10 - c00);  // interpolate along angle at m0
    float c1 = c01 + af * (c11 - c01);  // interpolate along angle at m0+1

    return c0 + mf * (c1 - c0);         // interpolate along Mach
}

float CdLookup::bodyCd(float mach) {
    return lookupCd(ANGLE_MIN, mach);
}

// Inverse lookup: given cd_add and Mach, find airbrake angle in degrees
// cd_add = desired total Cd - body Cd at this Mach
// Uses linear search + interpolation along the angle axis at the given Mach
static float inverseLookupAngle(float cd_add, float mach) {
    float body_cd = CdLookup::bodyCd(mach);
    float target_cd = body_cd + cd_add;

    // Clamp Mach for index computation
    float mach_clamped = mach;
    if (mach_clamped < MACH_MIN) mach_clamped = MACH_MIN;
    if (mach_clamped > MACH_MAX) mach_clamped = MACH_MAX;

    // Get Cd values at each angle for this Mach (interpolated between Mach rows)
    float mi = (mach_clamped - MACH_MIN) / MACH_STEP;
    int m0 = (int)mi;
    if (m0 >= MACH_COUNT - 1) m0 = MACH_COUNT - 2;
    float mf = mi - m0;

    // Search along angle axis
    for (int i = 0; i < ANGLE_COUNT - 1; i++) {
        float cd_lo = CD_TABLE[m0][i] + mf * (CD_TABLE[m0 + 1][i] - CD_TABLE[m0][i]);
        float cd_hi = CD_TABLE[m0][i + 1] + mf * (CD_TABLE[m0 + 1][i + 1] - CD_TABLE[m0][i + 1]);

        if (target_cd >= cd_lo && target_cd <= cd_hi) {
            // Interpolate angle
            float t = (target_cd - cd_lo) / (cd_hi - cd_lo);
            return ANGLE_MIN + (i + t) * ANGLE_STEP;
        }
    }

    // If target_cd is below minimum table value, return min angle
    // If above maximum, return max angle
    float cd_min = CD_TABLE[m0][0] + mf * (CD_TABLE[m0 + 1][0] - CD_TABLE[m0][0]);
    if (target_cd <= cd_min) return ANGLE_MIN;
    return ANGLE_MAX;
}

float CdLookup::angleDegToServoPct(float angle_deg) {
    // Linear mapping: 1° → SERVO_MIN_PCT, 95° → SERVO_MAX_PCT
    float t = (angle_deg - ANGLE_MIN) / (ANGLE_MAX - ANGLE_MIN);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return RocketConfig::SERVO_MIN_PCT + t * (RocketConfig::SERVO_MAX_PCT - RocketConfig::SERVO_MIN_PCT);
}

float CdLookup::servoPctToAngleDeg(float servo_pct) {
    float t = (servo_pct - RocketConfig::SERVO_MIN_PCT) /
              (RocketConfig::SERVO_MAX_PCT - RocketConfig::SERVO_MIN_PCT);
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    return ANGLE_MIN + t * (ANGLE_MAX - ANGLE_MIN);
}

ServoAngles CdLookup::cdToServoAngles(float cd_add, float mach) {
    if (cd_add < 0.0f) cd_add = 0.0f;
    if (cd_add > RocketConfig::MAX_CD_ADD) cd_add = RocketConfig::MAX_CD_ADD;

    float angle_deg = inverseLookupAngle(cd_add, mach);
    float servo_pct = angleDegToServoPct(angle_deg);

    ServoAngles result;
    result.angle_1 = servo_pct;
    result.angle_2 = servo_pct;  // Mirrored for now
    return result;
}
