#include "sp_app.h"

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* ---- minimal JSON readers --------------------------------------------------------------- */
static const char* find_value(const char* json, const char* key) {
    size_t klen = strlen(key);
    const char* p = json;
    while ((p = strchr(p, '"')) != NULL) {
        if (strncmp(p + 1, key, klen) == 0 && p[1 + klen] == '"') {
            const char* q = p + 2 + klen;
            while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') {
                q++;
            }
            if (*q == ':') {
                q++;
                while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') {
                    q++;
                }
                return q;
            }
        }
        p++;
    }
    return NULL;
}

int sp_json_string(const char* json, const char* key, char* out, size_t n) {
    const char* v = find_value(json, key);
    size_t i = 0;
    if (v == NULL || *v != '"' || n == 0) {
        return -1;
    }
    v++;
    while (*v && *v != '"' && i + 1 < n) {
        out[i++] = *v++;
    }
    out[i] = '\0';
    return *v == '"' ? 0 : -1;
}

int sp_json_number(const char* json, const char* key, float* out) {
    const char* v = find_value(json, key);
    char* end;
    if (v == NULL) {
        return -1;
    }
    double d = strtod(v, &end);
    if (end == v) {
        return -1;
    }
    *out = (float)d;
    return 0;
}

int sp_json_numbers(const char* json, const char* key, float* out, int max) {
    const char* v = find_value(json, key);
    int n = 0;
    if (v == NULL || *v != '[') {
        return -1;
    }
    v++;
    while (n < max) {
        char* end;
        while (*v == ' ' || *v == ',') {
            v++;
        }
        if (*v == ']') {
            break;
        }
        double d = strtod(v, &end);
        if (end == v) {
            return -1;
        }
        out[n++] = (float)d;
        v = end;
    }
    return n;
}

/* ---- app -------------------------------------------------------------------------------- */
void sp_app_init(sp_app_t* app, const sp_port_t* port, const uint8_t* node_ids, uint8_t n_legs, uint8_t master_node,
                 bool auto_home) {
    memset(app, 0, sizeof *app);
    sp_core_init(&app->core, port, node_ids, n_legs, master_node, auto_home);
    sp_motion_init(&app->motion);
    sp_motion_reset_level(&app->motion, SP_HOMING_PARK_MM);
    app->last_phase = app->core.phase;
}

void sp_app_step(sp_app_t* app, uint32_t dt_us) {
    sp_core_step(&app->core, dt_us);
    if (app->core.phase != app->last_phase) {
        if (app->core.phase == SP_PH_RUN && app->last_phase == SP_PH_HOMING) {
            sp_motion_reset_level(&app->motion, SP_HOMING_PARK_MM); /* rise smoothly from the park height */
        }
        app->last_phase = app->core.phase;
    }
}

void sp_app_on_sync(sp_app_t* app) {
    float legs[6];
    sp_motion_step(&app->motion, (float)SP_SYNC_PERIOD_MS / 1000.0f, legs);
    if (app->core.phase == SP_PH_RUN) {
        sp_core_set_leg_targets(&app->core, legs, 6);
    }
    sp_core_on_sync(&app->core);
}

int sp_app_command(sp_app_t* app, const char* json) {
    char type[16];
    float v[6];
    int rc = 0;
    app->commands++;
    if (sp_json_string(json, "type", type, sizeof type) != 0) {
        app->bad_commands++;
        return -1;
    }
    if (strcmp(type, "home") == 0) {
        sp_core_cmd_home(&app->core);
    } else if (strcmp(type, "stop") == 0) {
        sp_core_cmd_stop(&app->core, "stopped by operator");
    } else if (strcmp(type, "resume") == 0) {
        sp_core_cmd_resume(&app->core);
    } else if (strcmp(type, "source") == 0) {
        char name[16];
        sp_source_t s;
        if (sp_json_string(json, "source", name, sizeof name) == 0 && sp_source_parse(name, &s)) {
            sp_motion_set_source(&app->motion, s);
        } else {
            rc = -1;
        }
    } else if (strcmp(type, "pose") == 0) {
        static const char* const keys[6] = {"x", "y", "z", "roll", "pitch", "yaw"};
        sp_pose_t p = {0};
        for (int k = 0; k < 6; k++) {
            float f;
            if (sp_json_number(json, keys[k], &f) == 0) {
                *sp_pose_axis(&p, k) = f;
            }
        }
        sp_motion_set_pose(&app->motion, &p);
    } else if (strcmp(type, "joystick") == 0) {
        float x = 0.0f, y = 0.0f;
        (void)sp_json_number(json, "x", &x);
        (void)sp_json_number(json, "y", &y);
        sp_motion_set_joystick(&app->motion, x, y);
    } else if (strcmp(type, "imu") == 0) {
        float r = 0.0f, p = 0.0f, y = 0.0f;
        (void)sp_json_number(json, "roll", &r);
        (void)sp_json_number(json, "pitch", &p);
        (void)sp_json_number(json, "yaw", &y);
        sp_motion_set_imu(&app->motion, r, p, y);
    } else if (strcmp(type, "legs") == 0) {
        int n = sp_json_numbers(json, "mm", v, 6);
        if (n > 0) {
            sp_motion_set_legs(&app->motion, v, (uint8_t)n);
        } else {
            rc = -1;
        }
    } else if (strcmp(type, "setpoint") == 0) {
        if (sp_json_number(json, "mm", &v[0]) == 0) {
            for (int i = 1; i < 6; i++) {
                v[i] = v[0];
            }
            sp_motion_set_source(&app->motion, SP_SRC_LEGS);
            sp_motion_set_legs(&app->motion, v, 6);
        } else {
            rc = -1;
        }
    } else {
        rc = -1;
    }
    if (rc != 0) {
        app->bad_commands++;
    }
    return rc;
}

/* ---- status ----------------------------------------------------------------------------- */
typedef struct {
    char* buf;
    size_t n, len;
} out_t;

#if defined(__GNUC__)
static void put(out_t* o, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
#endif
static void put(out_t* o, const char* fmt, ...) {
    va_list ap;
    if (o->len >= o->n) {
        return;
    }
    va_start(ap, fmt);
    int w = vsnprintf(o->buf + o->len, o->n - o->len, fmt, ap);
    va_end(ap);
    if (w > 0) {
        o->len += (size_t)w;
        if (o->len > o->n) {
            o->len = o->n;
        }
    }
}

static void put_pose(out_t* o, const char* name, const sp_pose_t* p) {
    put(o, "\"%s\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f]", name, (double)p->x, (double)p->y, (double)p->z,
        (double)p->roll, (double)p->pitch, (double)p->yaw);
}

int sp_app_status_json(const sp_app_t* app, char* buf, size_t n) {
    const sp_core_t* c = &app->core;
    const sp_motion_t* m = &app->motion;
    out_t o = {buf, n, 0};
    if (n == 0) {
        return 0;
    }
    buf[0] = '\0';
    put(&o, "{\"source\":\"master\",\"impl\":\"c\",\"phase\":\"%s\",\"reason\":\"%s\",\"homed\":%s,", sp_phase_name(c->phase),
        c->reason, c->homed ? "true" : "false");
    put(&o, "\"input\":\"%s\",\"demo\":%s,\"t_ms\":%lu,\"syncs\":%lu,", sp_source_name(m->source),
        m->demo_active ? "true" : "false", (unsigned long)c->now_ms, (unsigned long)c->syncs);
    put(&o, "\"setpoint_mm\":%.3f,\"target_mm\":%.3f,", (double)c->legs[0].target_mm, (double)c->legs[0].x_mm);
    put_pose(&o, "pose_target", &m->target);
    put(&o, ",");
    put_pose(&o, "pose", &m->pose);
    put(&o, ",\"legs\":[");
    for (uint8_t i = 0; i < c->n_legs; i++) {
        const sp_leg_t* L = &c->legs[i];
        put(&o, "%s{\"node\":%u,\"boot\":\"%s\",\"nmt\":%d,\"hb\":%s,\"sw\":\"0x%04X\",\"state\":\"%s\",", i ? "," : "",
            L->node, sp_boot_name(L->boot), L->in.nmt, L->in.hb_active ? "true" : "false", L->in.statusword,
            sp_leg_state_name(L->in.statusword));
        put(&o, "\"actual_mm\":%.3f,\"target_mm\":%.3f,\"x_mm\":%.3f,\"load\":%u}", (double)sp_leg_actual_mm(c, i),
            (double)L->target_mm, (double)L->x_mm, L->load);
    }
    put(&o, "],\"emcy\":[");
    uint8_t count = c->emcy_n < SP_EMCY_LOG_LEN ? c->emcy_n : SP_EMCY_LOG_LEN;
    for (uint8_t k = 0; k < count; k++) {
        const sp_emcy_log_t* e = &c->emcy[(uint8_t)(c->emcy_n - count + k) % SP_EMCY_LOG_LEN];
        put(&o, "%s{\"age_s\":%.1f,\"node\":%u,\"code\":\"0x%04X\",\"byte1\":%u,\"class\":\"%s\"}", k ? "," : "",
            (double)(c->now_ms - e->t_ms) / 1000.0, e->node, e->code, e->byte1, sp_emcy_class_name(e->cls));
    }
    put(&o, "]}");
    return (int)o.len;
}
