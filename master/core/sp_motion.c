#include "sp_motion.h"

#include <math.h>
#include <string.h>

#include "sp_config_gen.h"

#define TWO_PI 6.283185307179586f

static const char* const SOURCE_NAMES[SP_SRC_COUNT] = {"pose", "joystick", "imu", "demo", "legs"};

const char* sp_source_name(sp_source_t s) { return (unsigned)s < SP_SRC_COUNT ? SOURCE_NAMES[s] : "?"; }

bool sp_source_parse(const char* name, sp_source_t* out) {
    for (int i = 0; i < SP_SRC_COUNT; i++) {
        if (strcmp(name, SOURCE_NAMES[i]) == 0) {
            *out = (sp_source_t)i;
            return true;
        }
    }
    return false;
}

static float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

void sp_motion_init(sp_motion_t* m) {
    memset(m, 0, sizeof *m);
    m->source = SP_SRC_POSE;
    for (int i = 0; i < 6; i++) {
        m->legs_cmd[i] = SP_NEUTRAL_STROKE_MM;
        m->legs[i] = SP_NEUTRAL_STROKE_MM;
    }
}

void sp_motion_set_source(sp_motion_t* m, sp_source_t s) {
    if ((unsigned)s < SP_SRC_COUNT) {
        m->source = s;
        m->idle_s = 0.0f;
        m->demo_t = 0.0f;
    }
}

void sp_motion_set_pose(sp_motion_t* m, const sp_pose_t* p) {
    m->pose_cmd = *p;
    m->idle_s = 0.0f;
}

void sp_motion_set_joystick(sp_motion_t* m, float x, float y) {
    float nx = clamp(x, -1.0f, 1.0f), ny = clamp(y, -1.0f, 1.0f);
    if (fabsf(nx - m->joy[0]) > SP_JOY_DEADBAND || fabsf(ny - m->joy[1]) > SP_JOY_DEADBAND) {
        m->idle_s = 0.0f;
    }
    m->joy[0] = nx;
    m->joy[1] = ny;
}

void sp_motion_set_imu(sp_motion_t* m, float roll, float pitch, float yaw) {
    m->imu[0] = roll;
    m->imu[1] = pitch;
    m->imu[2] = yaw;
    m->idle_s = 0.0f;
}

void sp_motion_set_legs(sp_motion_t* m, const float* mm, uint8_t count) {
    for (uint8_t i = 0; i < count && i < 6; i++) {
        m->legs_cmd[i] = mm[i];
    }
    m->idle_s = 0.0f;
}

void sp_motion_reset_level(sp_motion_t* m, float stroke_mm) {
    memset(&m->pose, 0, sizeof m->pose);
    m->pose.z = sp_kin_level_z(stroke_mm);
    memset(m->vel, 0, sizeof m->vel);
}

/* Deadband, then blend linear and cubic ("expo"). */
float sp_joystick_shape(float v) {
    float a = fabsf(v);
    if (a <= SP_JOY_DEADBAND) {
        return 0.0f;
    }
    a = (a - SP_JOY_DEADBAND) / (1.0f - SP_JOY_DEADBAND);
    a = clamp(a, 0.0f, 1.0f);
    a = (1.0f - SP_JOY_EXPO) * a + SP_JOY_EXPO * a * a * a;
    return v < 0.0f ? -a : a;
}

static sp_pose_t source_pose(sp_motion_t* m, float dt) {
    sp_pose_t p = {0};
    bool idle_demo = SP_DEMO_IDLE_AFTER_S > 0.0f && m->idle_s > SP_DEMO_IDLE_AFTER_S &&
                     (m->source == SP_SRC_JOYSTICK || m->source == SP_SRC_IMU);
    m->demo_active = m->source == SP_SRC_DEMO || idle_demo;
    if (m->demo_active) {
        float ph = TWO_PI * m->demo_t / SP_DEMO_PERIOD_S;
        m->demo_t += dt;
        p.roll = SP_DEMO_ROLL_DEG * sinf(ph);
        p.pitch = SP_DEMO_PITCH_DEG * cosf(ph);
        p.z = SP_DEMO_Z_MM * sinf(2.0f * ph);
        return p;
    }
    switch (m->source) {
        case SP_SRC_POSE:
            return m->pose_cmd;
        case SP_SRC_JOYSTICK:
            p.roll = SP_JOY_ROLL_DEG * sp_joystick_shape(m->joy[0]);
            p.pitch = SP_JOY_PITCH_DEG * sp_joystick_shape(m->joy[1]);
            return p;
        case SP_SRC_IMU:
            p.roll = clamp(SP_IMU_GAIN * m->imu[0], -SP_IMU_CLAMP_DEG, SP_IMU_CLAMP_DEG);
            p.pitch = clamp(SP_IMU_GAIN * m->imu[1], -SP_IMU_CLAMP_DEG, SP_IMU_CLAMP_DEG);
            p.yaw = SP_IMU_USE_YAW ? clamp(SP_IMU_GAIN * m->imu[2], -SP_IMU_CLAMP_DEG, SP_IMU_CLAMP_DEG) : 0.0f;
            return p;
        default:
            return p;
    }
}

/* Time-optimal-ish approach to the target: v toward target limited by sqrt(2 a |err|), vmax, amax. */
static void approach(float* x, float* v, float target, float vmax, float amax, float dt) {
    float err = target - *x;
    float v_cmd = sqrtf(2.0f * amax * fabsf(err));
    if (v_cmd > vmax) {
        v_cmd = vmax;
    }
    v_cmd = copysignf(v_cmd, err);
    *v = clamp(v_cmd, *v - amax * dt, *v + amax * dt);
    *x += *v * dt;
    if (fabsf(target - *x) < 1e-4f && fabsf(*v) < amax * dt) {
        *x = target;
        *v = 0.0f;
    }
}

void sp_motion_step(sp_motion_t* m, float dt, float out[6]) {
    m->idle_s += dt;
    if (m->source == SP_SRC_LEGS) {
        for (int i = 0; i < 6; i++) {
            m->legs[i] = out[i] = clamp(m->legs_cmd[i], SP_SOFT_MIN_MM, SP_SOFT_MAX_MM);
        }
        return;
    }
    sp_pose_t raw = source_pose(m, dt);
    m->target = sp_kin_limit(&raw);

    sp_pose_t prev = m->pose;
    float prev_vel[SP_POSE_AXES];
    memcpy(prev_vel, m->vel, sizeof prev_vel);
    for (int k = 0; k < SP_POSE_AXES; k++) {
        bool lin = k < 3;
        approach(sp_pose_axis(&m->pose, k), &m->vel[k], *sp_pose_axis(&m->target, k),
                 lin ? SP_POSE_MAX_LIN_VEL_MM_S : SP_POSE_MAX_ANG_VEL_DEG_S,
                 lin ? SP_POSE_MAX_LIN_ACC_MM_S2 : SP_POSE_MAX_ANG_ACC_DEG_S2, dt);
    }

    /* Keep every leg within the csp speed limit by shortening the pose step (keeps the path). */
    float s_prev[6], s_new[6], worst = 0.0f;
    sp_kin_strokes(&prev, s_prev);
    sp_kin_strokes(&m->pose, s_new);
    for (int i = 0; i < 6; i++) {
        float d = fabsf(s_new[i] - s_prev[i]);
        if (d > worst) {
            worst = d;
        }
    }
    float allowed = 0.95f * SP_CSP_MAX_VELOCITY_MM_S * dt;
    if (worst > allowed) {
        float f = allowed / worst;
        for (int k = 0; k < SP_POSE_AXES; k++) {
            float* a = sp_pose_axis(&m->pose, k);
            const float* b = sp_pose_axis(&prev, k);
            *a = *b + f * (*a - *b);
            m->vel[k] = prev_vel[k] + f * (m->vel[k] - prev_vel[k]);
            if (fabsf(m->vel[k]) > fabsf((*a - *b) / dt) + 1e-6f) {
                m->vel[k] = (*a - *b) / dt;
            }
        }
        sp_kin_strokes(&m->pose, s_new);
    }
    for (int i = 0; i < 6; i++) {
        m->legs[i] = out[i] = s_new[i];
    }
}
