#include "sp_kin.h"

#include <math.h>

#include "sp_config_gen.h"

#define DEG2RAD 0.017453292519943295f

float* sp_pose_axis(sp_pose_t* p, int axis) {
    switch (axis) {
        case 0: return &p->x;
        case 1: return &p->y;
        case 2: return &p->z;
        case 3: return &p->roll;
        case 4: return &p->pitch;
        default: return &p->yaw;
    }
}

void sp_kin_strokes(const sp_pose_t* p, float s[6]) {
    float cr = cosf(p->roll * DEG2RAD), sr = sinf(p->roll * DEG2RAD);
    float cp = cosf(p->pitch * DEG2RAD), sp = sinf(p->pitch * DEG2RAD);
    float cy = cosf(p->yaw * DEG2RAD), sy = sinf(p->yaw * DEG2RAD);
    float r[3][3] = {
        {cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr},
        {sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr},
        {-sp, cp * sr, cp * cr},
    };
    float t[3] = {p->x, p->y, p->z + SP_NEUTRAL_Z_MM};
    for (int i = 0; i < 6; i++) {
        const float* b = SP_BASE_JOINTS[i];
        const float* q = SP_PLATFORM_JOINTS[i];
        float d2 = 0.0f;
        for (int k = 0; k < 3; k++) {
            float v = t[k] + r[k][0] * q[0] + r[k][1] * q[1] + r[k][2] * q[2] - b[k];
            d2 += v * v;
        }
        s[i] = sqrtf(d2) - SP_LEG_CLOSED_MM;
    }
}

bool sp_kin_reachable(const sp_pose_t* p) {
    float s[6];
    sp_kin_strokes(p, s);
    for (int i = 0; i < 6; i++) {
        if (s[i] < SP_SOFT_MIN_MM + SP_WORKSPACE_MARGIN_MM || s[i] > SP_SOFT_MAX_MM - SP_WORKSPACE_MARGIN_MM) {
            return false;
        }
    }
    return true;
}

static float clampf(float v, float lim) { return v < -lim ? -lim : (v > lim ? lim : v); }

sp_pose_t sp_kin_limit(const sp_pose_t* in) {
    sp_pose_t p = {clampf(in->x, SP_LIMIT_X_MM),       clampf(in->y, SP_LIMIT_Y_MM),
                   clampf(in->z, SP_LIMIT_Z_MM),       clampf(in->roll, SP_LIMIT_ROLL_DEG),
                   clampf(in->pitch, SP_LIMIT_PITCH_DEG), clampf(in->yaw, SP_LIMIT_YAW_DEG)};
    if (sp_kin_reachable(&p)) {
        return p;
    }
    float lo = 0.0f, hi = 1.0f;
    for (int k = 0; k < 20; k++) {
        float mid = 0.5f * (lo + hi);
        sp_pose_t q = {p.x * mid, p.y * mid, p.z * mid, p.roll * mid, p.pitch * mid, p.yaw * mid};
        if (sp_kin_reachable(&q)) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    sp_pose_t out = {p.x * lo, p.y * lo, p.z * lo, p.roll * lo, p.pitch * lo, p.yaw * lo};
    return out;
}

float sp_kin_level_z(float stroke_mm) {
    const float* b = SP_BASE_JOINTS[0];
    const float* q = SP_PLATFORM_JOINTS[0];
    float dx = q[0] - b[0], dy = q[1] - b[1];
    float len = SP_LEG_CLOSED_MM + stroke_mm;
    return sqrtf(len * len - dx * dx - dy * dy) + b[2] - q[2] - SP_NEUTRAL_Z_MM;
}
