/* Stewart platform master core. See sp_core.h for the call pattern and contract/README.md for the
 * drive-side behaviour this implements (startup, homing, csp, EMCY classes). */
#include "sp_core.h"

#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#if defined(__GNUC__)
#define SP_PRINTF(a, b) __attribute__((format(printf, a, b)))
#else
#define SP_PRINTF(a, b)
#endif

/* Leg motion states (all legs move through these together during homing). */
enum {
    M_IDLE,
    M_HM_ZERO,        /* target velocity 0 + disable voltage: 60FFh persists across enables */
    M_HM_CURRENT,     /* 2003h = homing current */
    M_HM_ENABLE,      /* enable in pv */
    M_HM_GO,          /* start retracting */
    M_HM_TRAVEL,      /* wait: stall EMCY, drive fault, 2101h load, timeout */
    M_HM_SETTLE,      /* velocity 0, let TPDO1 report the post-stall state */
    M_HM_ENABLE_HM,   /* enable in homing mode (fault reset if the stall faulted the drive) */
    M_HM_SET,         /* controlword new set point: method 35 = here is home */
    M_HM_ACK,         /* wait homing attained */
    M_HM_RUN_CURRENT, /* 2003h = run current */
    M_HM_DONE,        /* wait for all legs */
    M_PARK_ENABLE,    /* enable in pp */
    M_PARK_PRE,       /* target, bit 4 low */
    M_PARK_SET,       /* bit 4 high */
    M_PARK_ACK,       /* wait set-point acknowledge, then bit 4 low */
    M_PARK_MOVE,      /* wait target reached at the park position */
    M_NMT_START,      /* resume after the drive went NMT stopped */
    M_CSP_ENABLE,     /* enable in csp */
    M_CSP             /* streaming */
};

enum { EN_START, EN_FAULT_RESET, EN_WAIT_SOD, EN_SHUTDOWN, EN_WAIT_RTSO, EN_WAIT_SO, EN_WAIT_OE };
enum { END_NONE, END_EMCY, END_FAULT, END_LOAD, END_TIMEOUT };
enum { BUSY = 0, DONE = 1, FAIL = -1 };

#define SDO_ABORT_TIMEOUT 0x05040000u
#define TRAVEL_IGNORE_LOAD_MS 300u
#define BOOT_RETRY_MS 2000u
#define NMT_START_TIMEOUT_MS 1000u
#define ACK_TIMEOUT_MS 1000u

static const char* const END_NAMES[] = {"-", "stallGuard EMCY 3230h", "drive fault", "stallGuard load", "timeout"};

/* ------------------------------------------------------------------------------------------ */
static uint32_t elapsed(const sp_core_t* c, uint32_t t0) { return c->now_ms - t0; }

static int32_t mm_to_counts(float mm) { return (int32_t)lroundf(mm * (float)SP_COUNTS_PER_MM); }

static bool is(uint16_t sw, uint16_t mask, uint16_t value) { return (uint16_t)(sw & mask) == value; }
#define IN_STATE(sw, name) is((sw), SP_SW_##name##_MASK, SP_SW_##name)

static void logf_(sp_core_t* c, int level, const char* fmt, ...) SP_PRINTF(3, 4);
static void logf_(sp_core_t* c, int level, const char* fmt, ...) {
    char buf[128];
    va_list ap;
    if (c->port.log == NULL) {
        return;
    }
    va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    c->port.log(c->port.ctx, level, buf);
}

static void set_reason(sp_core_t* c, const char* fmt, ...) SP_PRINTF(2, 3);
static void set_reason(sp_core_t* c, const char* fmt, ...) {
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(c->reason, sizeof c->reason, fmt, ap);
    va_end(ap);
}

static bool leg_active(const sp_core_t* c, uint8_t i) { return i < c->n_legs && c->legs[i].node != 0; }

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

/* ------------------------------------------------------------------------------------------ */
/* SDO: one outstanding transfer, owned by one leg. */
static int leg_sdo(sp_core_t* c, uint8_t i, bool write, uint16_t index, uint8_t sub, uint32_t value, uint8_t size) {
    sp_leg_t* L = &c->legs[i];
    if (L->sdo_done) {
        L->sdo_done = false;
        L->sdo_pending = false;
        if (L->sdo_abort != 0) {
            logf_(c, 0, "node %u: SDO %s %04Xh.%u aborted 0x%08lX", L->node, write ? "write" : "read", index, sub,
                  (unsigned long)L->sdo_abort);
            return FAIL;
        }
        return DONE;
    }
    if (L->sdo_pending) {
        if (elapsed(c, c->sdo_t0) > 4u * SP_TIMEOUT_SDO_MS) { /* adapter never answered */
            L->sdo_pending = false;
            L->sdo_abort = SDO_ABORT_TIMEOUT;
            c->sdo_busy = false;
            logf_(c, 0, "node %u: SDO %04Xh.%u: no result from the adapter", L->node, index, sub);
            return FAIL;
        }
        return BUSY;
    }
    if (c->sdo_busy) {
        return BUSY;
    }
    if (!(write ? c->port.sdo_write(c->port.ctx, L->node, index, sub, value, size)
                : c->port.sdo_read(c->port.ctx, L->node, index, sub))) {
        return BUSY; /* client not available: try again next step */
    }
    c->sdo_busy = true;
    c->sdo_t0 = c->now_ms;
    L->sdo_pending = true;
    L->sdo_done = false;
    return BUSY;
}

void sp_core_sdo_done(sp_core_t* c, uint32_t abort_code, uint32_t value) {
    for (uint8_t i = 0; i < SP_MAX_LEGS; i++) {
        sp_leg_t* L = &c->legs[i];
        if (L->sdo_pending) {
            L->sdo_done = true;
            L->sdo_abort = abort_code;
            L->sdo_value = value;
        }
    }
    c->sdo_busy = false;
}

/* ------------------------------------------------------------------------------------------ */
/* Drive bring-up (per leg; legs share the SDO client and take turns). */
static void boot_error(sp_core_t* c, uint8_t i, const char* why) {
    sp_leg_t* L = &c->legs[i];
    logf_(c, 0, "node %u: %s", L->node, why);
    set_reason(c, "node %u: %s", L->node, why);
    L->boot = SP_BOOT_ERROR;
    L->boot_t0 = c->now_ms;
}

static void leg_boot_step(sp_core_t* c, uint8_t i) {
    sp_leg_t* L = &c->legs[i];
    int r;
    switch (L->boot) {
        case SP_BOOT_RESET:
            c->port.nmt(c->port.ctx, SP_NMT_RESET_NODE, L->node);
            L->bootup = false;
            L->boot = SP_BOOT_WAIT;
            L->boot_t0 = c->now_ms;
            break;
        case SP_BOOT_WAIT:
            if (L->bootup) {
                L->bootup = false;
                L->boot = SP_BOOT_IDENTIFY;
                L->boot_sub = 0;
            } else if (elapsed(c, L->boot_t0) > SP_TIMEOUT_BOOTUP_MS) {
                boot_error(c, i, "no boot-up");
            }
            break;
        case SP_BOOT_IDENTIFY: {
            static const uint8_t subs[3] = {1, 2, 3};
            r = leg_sdo(c, i, false, 0x1018, subs[L->boot_sub], 0, 4);
            if (r == FAIL) {
                boot_error(c, i, "identity read failed");
            } else if (r == DONE) {
                uint32_t v = L->sdo_value;
                if (L->boot_sub == 0 && v != SP_IDENTITY_VENDOR) {
                    boot_error(c, i, "unexpected vendor id");
                } else if (L->boot_sub == 1 && v != SP_IDENTITY_PRODUCT) {
                    boot_error(c, i, "unexpected product code");
                } else if (L->boot_sub == 2) {
                    L->revision = v;
                    if (v != 0 && v < SP_IDENTITY_MIN_REVISION) {
                        logf_(c, 1, "node %u: firmware revision 0x%08lX older than the contract", L->node,
                              (unsigned long)v);
                    }
                    L->boot = SP_BOOT_PREPARE;
                    L->boot_sub = 0;
                } else {
                    L->boot_sub++;
                }
            }
            break;
        }
        case SP_BOOT_PREPARE:
            /* 2005h and 208Ch are only writable in SWITCH ON DISABLED. */
            if (L->boot_sub == 0) {
                r = leg_sdo(c, i, false, 0x6041, 0, 0, 2);
                if (r == DONE) {
                    uint16_t sw = (uint16_t)L->sdo_value;
                    L->boot_sub = IN_STATE(sw, FAULT) ? 1 : (IN_STATE(sw, SWITCH_ON_DISABLED) ? 9 : 3);
                }
            } else if (L->boot_sub == 1) {
                r = leg_sdo(c, i, true, 0x6040, 0, 0, 2);
                if (r == DONE) {
                    L->boot_sub = 2;
                }
            } else if (L->boot_sub == 2) {
                r = leg_sdo(c, i, true, 0x6040, 0, SP_CW_FAULT_RESET, 2);
                if (r == DONE) {
                    L->boot_sub = 9;
                }
            } else {
                r = leg_sdo(c, i, true, 0x6040, 0, SP_CW_DISABLE_VOLTAGE, 2);
                if (r == DONE) {
                    L->boot_sub = 9;
                }
            }
            if (r == FAIL) {
                boot_error(c, i, "could not reach SWITCH ON DISABLED");
            } else if (L->boot_sub == 9) {
                L->boot = SP_BOOT_CONFIGURE;
                L->boot_i = 0;
            }
            break;
        case SP_BOOT_CONFIGURE: {
            const sp_sdo_entry_t* e = &SP_STARTUP_SDO[L->boot_i];
            uint32_t v = e->value;
            if (e->flags & SP_SDO_ADD_NODE) {
                v += L->node;
            }
            if (e->flags & SP_SDO_ADD_MASTER_HB) {
                v += (uint32_t)c->master_node << 16;
            }
            r = leg_sdo(c, i, true, e->index, e->sub, v, e->size);
            if (r == FAIL) {
                char why[48];
                snprintf(why, sizeof why, "config write %04Xh.%u rejected", e->index, e->sub);
                boot_error(c, i, why);
            } else if (r == DONE && ++L->boot_i >= SP_STARTUP_SDO_COUNT) {
                logf_(c, 2, "node %u: configured (%u SDO writes)", L->node, (unsigned)SP_STARTUP_SDO_COUNT);
                L->boot = SP_BOOT_START;
                L->boot_sub = 0;
            }
            break;
        }
        case SP_BOOT_START:
            if (L->boot_sub == 0) {
                c->port.nmt(c->port.ctx, SP_NMT_START, L->node);
                L->boot_sub = 1;
                L->boot_t0 = c->now_ms;
            } else if (L->in.hb_active && L->in.nmt == SP_NMTS_OPERATIONAL) {
                L->boot = SP_BOOT_READY;
                L->bootup = false;
            } else if (elapsed(c, L->boot_t0) > NMT_START_TIMEOUT_MS) {
                boot_error(c, i, "did not go OPERATIONAL");
            }
            break;
        case SP_BOOT_LOST:
            if (L->in.hb_active) {
                if (L->bootup || L->in.nmt == SP_NMTS_BOOTUP || L->in.nmt == SP_NMTS_PREOP) {
                    logf_(c, 1, "node %u: back after a reboot: reconfiguring", L->node);
                    L->bootup = false;
                    L->boot = SP_BOOT_IDENTIFY;
                    L->boot_sub = 0;
                } else {
                    logf_(c, 1, "node %u: heartbeat back (no reboot)", L->node);
                    L->boot = SP_BOOT_READY;
                }
            }
            break;
        case SP_BOOT_ERROR:
            if (elapsed(c, L->boot_t0) > BOOT_RETRY_MS) {
                L->boot = SP_BOOT_RESET;
            }
            break;
        case SP_BOOT_OFF:
        case SP_BOOT_READY:
        default:
            break;
    }
}

/* ------------------------------------------------------------------------------------------ */
/* Motion helpers. At most one frame per drive RPDO per step: CANopenNode collapses repeated send
 * requests on one TPDO into a single frame. */
static void rpdo1(sp_core_t* c, uint8_t i, uint16_t cw, int32_t pos) { c->port.rpdo1(c->port.ctx, i, cw, pos); }
static void rpdo2(sp_core_t* c, uint8_t i, uint16_t cw, int8_t mode) { c->port.rpdo2(c->port.ctx, i, cw, mode); }
static void rpdo3(sp_core_t* c, uint8_t i, uint16_t cw, int32_t vel) { c->port.rpdo3(c->port.ctx, i, cw, vel); }

static void set_motion(sp_core_t* c, uint8_t i, uint8_t m) {
    sp_leg_t* L = &c->legs[i];
    L->motion = m;
    L->motion_t0 = c->now_ms;
    L->en = EN_START;
}

/* CiA 402 enable in a mode, mirroring the Python test master: fault reset if needed, then
 * shutdown -> switch on -> enable operation, each confirmed by the statusword. */
static int leg_enable(sp_core_t* c, uint8_t i, int8_t mode) {
    sp_leg_t* L = &c->legs[i];
    uint16_t sw = L->in.statusword;
    bool timeout = elapsed(c, L->en_t0) > SP_TIMEOUT_STATE_MS;
    switch (L->en) {
        case EN_START:
            L->en_mode = mode;
            L->en_t0 = c->now_ms;
            if (IN_STATE(sw, FAULT) || IN_STATE(sw, FAULT_REACTION_ACTIVE)) {
                rpdo2(c, i, 0x0000, mode);
                L->en = EN_FAULT_RESET;
            } else {
                L->en = EN_SHUTDOWN;
            }
            return BUSY;
        case EN_FAULT_RESET:
            rpdo2(c, i, SP_CW_FAULT_RESET, mode);
            L->en = EN_WAIT_SOD;
            L->en_t0 = c->now_ms;
            return BUSY;
        case EN_WAIT_SOD:
            if (IN_STATE(sw, SWITCH_ON_DISABLED)) {
                L->en = EN_SHUTDOWN;
            } else if (timeout) {
                logf_(c, 0, "node %u: fault reset failed (statusword 0x%04X)", L->node, sw);
                return FAIL;
            }
            return BUSY;
        case EN_SHUTDOWN:
            rpdo2(c, i, SP_CW_SHUTDOWN, mode);
            L->en = EN_WAIT_RTSO;
            L->en_t0 = c->now_ms;
            return BUSY;
        case EN_WAIT_RTSO:
            if (IN_STATE(sw, READY_TO_SWITCH_ON)) {
                rpdo2(c, i, SP_CW_SWITCH_ON, mode);
                L->en = EN_WAIT_SO;
                L->en_t0 = c->now_ms;
            } else if (timeout) {
                logf_(c, 0, "node %u: no READY TO SWITCH ON (statusword 0x%04X)", L->node, sw);
                return FAIL;
            }
            return BUSY;
        case EN_WAIT_SO:
            if (IN_STATE(sw, SWITCHED_ON)) {
                rpdo2(c, i, SP_CW_ENABLE_OPERATION, mode);
                L->en = EN_WAIT_OE;
                L->en_t0 = c->now_ms;
            } else if (timeout) {
                logf_(c, 0, "node %u: no SWITCHED ON (statusword 0x%04X)", L->node, sw);
                return FAIL;
            }
            return BUSY;
        case EN_WAIT_OE:
            if (IN_STATE(sw, OPERATION_ENABLED) && L->in.mode_display == mode) {
                L->en = EN_START;
                return DONE;
            }
            if (timeout) {
                logf_(c, 0, "node %u: no OPERATION ENABLED in mode %d (statusword 0x%04X, mode %d)", L->node, mode,
                      sw, L->in.mode_display);
                return FAIL;
            }
            return BUSY;
        default:
            return FAIL;
    }
}

/* Quick-stop every leg and leave the phase to the caller's choice. */
static void stop_all(sp_core_t* c, bool lose_homing, const char* why) {
    bool homing = c->phase == SP_PH_HOMING;
    for (uint8_t i = 0; i < c->n_legs; i++) {
        sp_leg_t* L = &c->legs[i];
        if (!leg_active(c, i)) {
            continue;
        }
        if (L->boot == SP_BOOT_READY && L->in.nmt == SP_NMTS_OPERATIONAL && L->motion != M_IDLE) {
            rpdo1(c, i, SP_CW_QUICK_STOP, L->in.position);
        }
        L->motion = M_IDLE;
        L->v_mm_s = 0.0f;
    }
    if (c->phase == SP_PH_BOOT || (c->phase == SP_PH_IDLE && !c->homed)) {
        set_reason(c, "%s", why);
        return; /* nothing was moving */
    }
    if (lose_homing || homing || !c->homed) {
        c->homed = false;
        c->phase = SP_PH_FAULT;
        set_reason(c, "%s: press Home", why);
    } else {
        c->phase = SP_PH_STOPPED;
        set_reason(c, "%s: press Resume", why);
    }
    logf_(c, 1, "stop all: %s", c->reason);
}

/* ------------------------------------------------------------------------------------------ */
/* Homing, per leg. Returns FAIL if the leg can't complete. */
static int leg_homing_step(sp_core_t* c, uint8_t i) {
    sp_leg_t* L = &c->legs[i];
    uint16_t sw = L->in.statusword;
    int r;
    switch (L->motion) {
        case M_HM_ZERO:
            if (L->in.nmt == SP_NMTS_OPERATIONAL) {
                rpdo3(c, i, SP_CW_DISABLE_VOLTAGE, 0);
            }
            set_motion(c, i, M_HM_CURRENT);
            return BUSY;
        case M_HM_CURRENT:
            r = leg_sdo(c, i, true, 0x2003, 0, SP_CURRENT_HOMING, 1);
            if (r == DONE) {
                set_motion(c, i, M_HM_ENABLE);
            }
            return r == FAIL ? FAIL : BUSY;
        case M_HM_ENABLE:
            r = leg_enable(c, i, SP_MODE_PROFILE_VELOCITY);
            if (r == DONE) {
                set_motion(c, i, M_HM_GO);
            }
            return r == FAIL ? FAIL : BUSY;
        case M_HM_GO:
            L->stall_emcy = false;
            L->end_reason = END_NONE;
            L->load = 0;
            rpdo3(c, i, SP_CW_ENABLE_OPERATION, -mm_to_counts(SP_HOMING_VELOCITY_MM_S));
            set_motion(c, i, M_HM_TRAVEL);
            L->poll_t0 = c->now_ms;
            return BUSY;
        case M_HM_TRAVEL:
            if (L->sdo_pending || (L->end_reason == END_NONE && elapsed(c, L->motion_t0) > TRAVEL_IGNORE_LOAD_MS &&
                                   elapsed(c, L->poll_t0) >= SP_HOMING_LOAD_POLL_MS)) {
                r = leg_sdo(c, i, false, 0x2101, 0, 0, 2);
                if (r == DONE) {
                    L->poll_t0 = c->now_ms;
                    L->load = (uint16_t)L->sdo_value;
                    if (L->load <= SP_HOMING_LOAD_STALL && L->end_reason == END_NONE) {
                        L->end_reason = END_LOAD;
                    }
                } else if (r == FAIL) {
                    L->poll_t0 = c->now_ms; /* keep travelling; the timeout still guarantees the stop */
                }
            }
            if (L->end_reason == END_NONE) {
                if (L->stall_emcy) {
                    L->end_reason = END_EMCY;
                } else if (IN_STATE(sw, FAULT) || IN_STATE(sw, FAULT_REACTION_ACTIVE)) {
                    L->end_reason = END_FAULT;
                } else if (elapsed(c, L->motion_t0) > SP_HOMING_TIMEOUT_MS) {
                    L->end_reason = END_TIMEOUT; /* blind overtravel: the stop is reached by now */
                }
            }
            if (L->end_reason != END_NONE && !L->sdo_pending) { /* finish an outstanding load read first */
                rpdo3(c, i, SP_CW_ENABLE_OPERATION, 0);
                logf_(c, 2, "node %u: end stop (%s) after %lu ms", L->node, END_NAMES[L->end_reason],
                      (unsigned long)elapsed(c, L->motion_t0));
                set_motion(c, i, M_HM_SETTLE);
            }
            return BUSY;
        case M_HM_SETTLE:
            if (elapsed(c, L->motion_t0) >= 3u * SP_SYNC_PERIOD_MS) {
                set_motion(c, i, M_HM_ENABLE_HM);
            }
            return BUSY;
        case M_HM_ENABLE_HM:
            r = leg_enable(c, i, SP_MODE_HOMING);
            if (r == DONE) {
                set_motion(c, i, M_HM_SET);
            }
            return r == FAIL ? FAIL : BUSY;
        case M_HM_SET:
            rpdo2(c, i, SP_CW_ENABLE_OPERATION | SP_CW_NEW_SET_POINT, SP_MODE_HOMING);
            set_motion(c, i, M_HM_ACK);
            return BUSY;
        case M_HM_ACK:
            if (sw & SP_SWB_SET_POINT_ACKNOWLEDGED) { /* in homing mode: homing attained */
                rpdo2(c, i, SP_CW_ENABLE_OPERATION, SP_MODE_HOMING);
                set_motion(c, i, M_HM_RUN_CURRENT);
            } else if (elapsed(c, L->motion_t0) > ACK_TIMEOUT_MS) {
                logf_(c, 0, "node %u: homing not attained", L->node);
                return FAIL;
            }
            return BUSY;
        case M_HM_RUN_CURRENT:
            r = leg_sdo(c, i, true, 0x2003, 0, SP_CURRENT_RUN, 1);
            if (r == DONE) {
                set_motion(c, i, M_HM_DONE);
            }
            return r == FAIL ? FAIL : BUSY;
        case M_HM_DONE:
            return BUSY;
        case M_PARK_ENABLE:
            r = leg_enable(c, i, SP_MODE_PROFILE_POSITION);
            if (r == DONE) {
                set_motion(c, i, M_PARK_PRE);
            }
            return r == FAIL ? FAIL : BUSY;
        case M_PARK_PRE:
            rpdo1(c, i, SP_CW_ENABLE_OPERATION, mm_to_counts(SP_HOMING_PARK_MM));
            set_motion(c, i, M_PARK_SET);
            return BUSY;
        case M_PARK_SET:
            rpdo1(c, i, SP_CW_ENABLE_OPERATION | SP_CW_NEW_SET_POINT, mm_to_counts(SP_HOMING_PARK_MM));
            set_motion(c, i, M_PARK_ACK);
            return BUSY;
        case M_PARK_ACK:
            if (sw & SP_SWB_SET_POINT_ACKNOWLEDGED) {
                rpdo1(c, i, SP_CW_ENABLE_OPERATION, mm_to_counts(SP_HOMING_PARK_MM));
                set_motion(c, i, M_PARK_MOVE);
            } else if (elapsed(c, L->motion_t0) > ACK_TIMEOUT_MS) {
                logf_(c, 0, "node %u: park set-point not acknowledged", L->node);
                return FAIL;
            }
            return BUSY;
        case M_PARK_MOVE: {
            float travel_s = SP_HOMING_PARK_MM / SP_PROFILE_VELOCITY_MM_S +
                             2.0f * SP_PROFILE_VELOCITY_MM_S / SP_PROFILE_ACCELERATION_MM_S2;
            if ((sw & SP_SWB_TARGET_REACHED) && L->in.position == mm_to_counts(SP_HOMING_PARK_MM)) {
                L->x_mm = L->target_mm = SP_HOMING_PARK_MM;
                L->v_mm_s = 0.0f;
                set_motion(c, i, M_CSP_ENABLE);
            } else if (elapsed(c, L->motion_t0) > (uint32_t)(travel_s * 1000.0f) + 2000u) {
                logf_(c, 0, "node %u: park position not reached", L->node);
                return FAIL;
            }
            return BUSY;
        }
        default:
            return BUSY;
    }
}

static int leg_csp_enable_step(sp_core_t* c, uint8_t i) {
    sp_leg_t* L = &c->legs[i];
    int r;
    if (L->motion == M_NMT_START) {
        if (L->en == EN_START) {
            c->port.nmt(c->port.ctx, SP_NMT_START, L->node);
            L->en = EN_WAIT_OE; /* reused as "waiting" marker */
            L->en_t0 = c->now_ms;
        } else if (L->in.nmt == SP_NMTS_OPERATIONAL) {
            set_motion(c, i, M_CSP_ENABLE);
        } else if (elapsed(c, L->en_t0) > NMT_START_TIMEOUT_MS) {
            logf_(c, 0, "node %u: did not go OPERATIONAL", L->node);
            return FAIL;
        }
        return BUSY;
    }
    if (L->motion == M_CSP_ENABLE) {
        r = leg_enable(c, i, SP_MODE_CYCLIC_SYNC_POSITION);
        if (r == DONE) {
            set_motion(c, i, M_CSP);
        }
        return r;
    }
    return DONE;
}

/* ------------------------------------------------------------------------------------------ */
static void start_homing(sp_core_t* c) {
    c->home_requested = false;
    c->homed = false;
    c->phase = SP_PH_HOMING;
    set_reason(c, "homing: retracting to the end stops");
    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (leg_active(c, i)) {
            set_motion(c, i, M_HM_ZERO);
        }
    }
    logf_(c, 2, "homing %u legs", c->n_legs);
}

static bool all_legs(const sp_core_t* c, uint8_t motion) {
    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (leg_active(c, i) && c->legs[i].motion != motion) {
            return false;
        }
    }
    return true;
}

static bool all_ready(const sp_core_t* c) {
    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (leg_active(c, i) && c->legs[i].boot != SP_BOOT_READY) {
            return false;
        }
    }
    return true;
}

static void homing_step(sp_core_t* c) {
    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (!leg_active(c, i)) {
            continue;
        }
        sp_leg_t* L = &c->legs[i];
        int r = (L->motion == M_CSP_ENABLE) ? leg_csp_enable_step(c, i) : leg_homing_step(c, i);
        if (r == FAIL) {
            char why[48];
            snprintf(why, sizeof why, "homing failed on node %u", L->node);
            stop_all(c, true, why);
            return;
        }
    }
    if (all_legs(c, M_HM_DONE)) {
        set_reason(c, "homing: parking at %.1f mm", (double)SP_HOMING_PARK_MM);
        for (uint8_t i = 0; i < c->n_legs; i++) {
            if (leg_active(c, i)) {
                set_motion(c, i, M_PARK_ENABLE);
            }
        }
    } else if (all_legs(c, M_CSP)) {
        c->homed = true;
        c->phase = SP_PH_RUN;
        set_reason(c, "following");
        logf_(c, 2, "homed: following targets in csp");
    }
}

static void run_step(sp_core_t* c) {
    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (!leg_active(c, i)) {
            continue;
        }
        sp_leg_t* L = &c->legs[i];
        if (L->motion == M_NMT_START || L->motion == M_CSP_ENABLE) {
            if (leg_csp_enable_step(c, i) == FAIL) {
                char why[48];
                snprintf(why, sizeof why, "node %u could not be enabled", L->node);
                stop_all(c, false, why);
                return;
            }
        } else if (L->motion == M_CSP && !IN_STATE(L->in.statusword, OPERATION_ENABLED)) {
            char why[48];
            snprintf(why, sizeof why, "node %u left OPERATION ENABLED", L->node);
            stop_all(c, false, why);
            return;
        }
    }
}

/* Drive lost, rebooted or stopped while configured. */
static void monitor_legs(sp_core_t* c) {
    for (uint8_t i = 0; i < c->n_legs; i++) {
        sp_leg_t* L = &c->legs[i];
        char why[48];
        if (!leg_active(c, i) || L->boot != SP_BOOT_READY) {
            continue;
        }
        if (L->bootup || (L->in.hb_active && (L->in.nmt == SP_NMTS_BOOTUP || L->in.nmt == SP_NMTS_PREOP))) {
            snprintf(why, sizeof why, "node %u rebooted", L->node);
            L->bootup = false;
            L->boot = SP_BOOT_IDENTIFY;
            L->boot_sub = 0;
            stop_all(c, true, why);
        } else if (!L->in.hb_active) {
            snprintf(why, sizeof why, "node %u heartbeat lost", L->node);
            L->boot = SP_BOOT_LOST;
            stop_all(c, false, why);
        }
    }
}

/* ------------------------------------------------------------------------------------------ */
void sp_core_init(sp_core_t* c, const sp_port_t* port, const uint8_t* node_ids, uint8_t n_legs, uint8_t master_node,
                  bool auto_home) {
    memset(c, 0, sizeof *c);
    c->port = *port;
    c->n_legs = n_legs > SP_MAX_LEGS ? SP_MAX_LEGS : n_legs;
    c->master_node = master_node;
    c->auto_home = auto_home;
    c->phase = SP_PH_BOOT;
    set_reason(c, "booting drives");
    for (uint8_t i = 0; i < SP_MAX_LEGS; i++) {
        sp_leg_t* L = &c->legs[i];
        L->node = i < c->n_legs ? node_ids[i] : 0;
        L->in.nmt = SP_NMTS_UNKNOWN;
        L->boot = leg_active(c, i) ? SP_BOOT_RESET : SP_BOOT_OFF;
        L->target_mm = L->x_mm = SP_HOMING_PARK_MM;
    }
}

sp_leg_in_t* sp_core_leg_in(sp_core_t* c, uint8_t leg) { return &c->legs[leg < SP_MAX_LEGS ? leg : 0].in; }

void sp_core_step(sp_core_t* c, uint32_t dt_us) {
    c->us_acc += dt_us;
    c->now_ms += c->us_acc / 1000u;
    c->us_acc %= 1000u;

    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (leg_active(c, i)) {
            leg_boot_step(c, i);
        }
    }
    monitor_legs(c);

    switch (c->phase) {
        case SP_PH_BOOT:
            if (all_ready(c)) {
                c->phase = SP_PH_IDLE;
                set_reason(c, "drives configured: press Home");
                logf_(c, 2, "%u drives configured", c->n_legs);
            }
            break;
        case SP_PH_HOMING:
            homing_step(c);
            break;
        case SP_PH_RUN:
            run_step(c);
            break;
        case SP_PH_IDLE:
        case SP_PH_STOPPED:
        case SP_PH_FAULT:
        default:
            break;
    }
    if (c->phase != SP_PH_HOMING && c->phase != SP_PH_RUN && c->phase != SP_PH_BOOT &&
        (c->home_requested || (c->auto_home && c->phase == SP_PH_IDLE))) {
        if (all_ready(c)) {
            c->auto_home = false; /* only once at start-up */
            start_homing(c);
        } else {
            c->phase = SP_PH_BOOT;
            set_reason(c, "booting drives");
        }
    }
}

static void trajectory(sp_leg_t* L, float dt) {
    const float vmax = SP_CSP_MAX_VELOCITY_MM_S, amax = SP_CSP_MAX_ACCELERATION_MM_S2;
    float err = L->target_mm - L->x_mm;
    float v_cmd = sqrtf(2.0f * amax * fabsf(err));
    if (v_cmd > vmax) {
        v_cmd = vmax;
    }
    v_cmd = copysignf(v_cmd, err);
    L->v_mm_s = clampf(v_cmd, L->v_mm_s - amax * dt, L->v_mm_s + amax * dt);
    L->x_mm += L->v_mm_s * dt;
    if (fabsf(L->target_mm - L->x_mm) < 1e-4f && fabsf(L->v_mm_s) < amax * dt) {
        L->x_mm = L->target_mm;
        L->v_mm_s = 0.0f;
    }
}

void sp_core_on_sync(sp_core_t* c) {
    c->syncs++;
    if (c->phase != SP_PH_RUN) {
        return;
    }
    for (uint8_t i = 0; i < c->n_legs; i++) {
        sp_leg_t* L = &c->legs[i];
        if (leg_active(c, i) && L->motion == M_CSP) {
            trajectory(L, (float)SP_SYNC_PERIOD_MS / 1000.0f);
            rpdo1(c, i, SP_CW_ENABLE_OPERATION, mm_to_counts(L->x_mm));
        }
    }
}

void sp_core_on_bootup(sp_core_t* c, uint8_t node) {
    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (c->legs[i].node == node) {
            c->legs[i].bootup = true;
        }
    }
}

uint8_t sp_emcy_classify(uint16_t code, uint8_t byte1) {
    int fallback = -1;
    for (unsigned k = 0; k < SP_EMCY_COUNT; k++) {
        const sp_emcy_entry_t* e = &SP_EMCY_TABLE[k];
        if (e->code != code) {
            continue;
        }
        if (e->byte1 < 0) {
            fallback = e->cls;
        } else if (e->byte1 == byte1) {
            return e->cls;
        }
    }
    return fallback < 0 ? (uint8_t)SP_EMCY_STOP_ALL : (uint8_t)fallback; /* unknown: conservative */
}

void sp_core_on_emcy(sp_core_t* c, uint8_t node, uint16_t code, uint8_t reg, uint8_t byte1) {
    int leg = -1;
    char why[48];
    (void)reg;
    for (uint8_t i = 0; i < c->n_legs; i++) {
        if (c->legs[i].node == node) {
            leg = i;
        }
    }
    uint8_t cls = sp_emcy_classify(code, byte1);
    sp_emcy_log_t* e = &c->emcy[c->emcy_n % SP_EMCY_LOG_LEN];
    e->t_ms = c->now_ms;
    e->node = node;
    e->code = code;
    e->byte1 = byte1;
    e->cls = cls;
    c->emcy_n++;
    logf_(c, cls <= SP_EMCY_WARN ? 2 : 1, "EMCY node %u 0x%04X/%u -> %s", node, code, byte1, sp_emcy_class_name(cls));
    if (leg < 0) {
        return;
    }
    sp_leg_t* L = &c->legs[leg];
    snprintf(why, sizeof why, "EMCY 0x%04X on node %u (%s)", code, node, sp_emcy_class_name(cls));
    switch (cls) {
        case SP_EMCY_INFO:
        case SP_EMCY_WARN:
            break;
        case SP_EMCY_HOMING_STALL:
            if (c->phase == SP_PH_HOMING && L->motion >= M_HM_ZERO && L->motion <= M_HM_DONE) {
                L->stall_emcy = true;
                break;
            }
            stop_all(c, true, why);
            break;
        case SP_EMCY_STOP_ALL:
            stop_all(c, false, why);
            break;
        case SP_EMCY_STALL:
        case SP_EMCY_STOP_ALL_REHOME:
        case SP_EMCY_CONFIG_ERROR:
        default:
            stop_all(c, true, why);
            break;
    }
}

void sp_core_set_leg_targets(sp_core_t* c, const float* mm, uint8_t count) {
    for (uint8_t i = 0; i < count && i < SP_MAX_LEGS; i++) {
        c->legs[i].target_mm = clampf(mm[i], SP_SOFT_MIN_MM, SP_SOFT_MAX_MM);
    }
}

bool sp_core_cmd_home(sp_core_t* c) {
    if (c->phase == SP_PH_HOMING) {
        return false;
    }
    if (c->phase == SP_PH_RUN) {
        stop_all(c, true, "re-homing");
    }
    for (uint8_t i = 0; i < c->n_legs; i++) {
        sp_leg_t* L = &c->legs[i];
        if (leg_active(c, i) && (L->boot == SP_BOOT_ERROR || L->boot == SP_BOOT_LOST)) {
            L->boot = SP_BOOT_RESET;
        }
    }
    c->home_requested = true;
    return true;
}

void sp_core_cmd_stop(sp_core_t* c, const char* why) {
    if (c->phase == SP_PH_RUN || c->phase == SP_PH_HOMING) {
        stop_all(c, false, why ? why : "stopped by operator");
    }
    c->home_requested = false;
}

bool sp_core_cmd_resume(sp_core_t* c) {
    if (c->phase != SP_PH_STOPPED || !c->homed || !all_ready(c)) {
        return false;
    }
    for (uint8_t i = 0; i < c->n_legs; i++) {
        sp_leg_t* L = &c->legs[i];
        if (!leg_active(c, i)) {
            continue;
        }
        L->x_mm = sp_leg_actual_mm(c, i);
        L->v_mm_s = 0.0f;
        set_motion(c, i, L->in.nmt == SP_NMTS_OPERATIONAL ? M_CSP_ENABLE : M_NMT_START);
    }
    c->phase = SP_PH_RUN;
    set_reason(c, "following");
    return true;
}

/* ------------------------------------------------------------------------------------------ */
const char* sp_phase_name(sp_phase_t p) {
    static const char* const n[] = {"boot", "idle", "homing", "run", "stopped", "fault"};
    return (unsigned)p < sizeof n / sizeof n[0] ? n[p] : "?";
}

const char* sp_boot_name(sp_boot_t b) {
    static const char* const n[] = {"off", "reset", "wait boot-up", "identify", "prepare", "configure", "start",
                                    "ready", "lost", "error"};
    return (unsigned)b < sizeof n / sizeof n[0] ? n[b] : "?";
}

const char* sp_emcy_class_name(uint8_t cls) {
    static const char* const n[] = {"info", "warn", "homing_stall", "stall", "stop_all", "stop_all_rehome",
                                    "config_error"};
    return cls < sizeof n / sizeof n[0] ? n[cls] : "?";
}

const char* sp_leg_state_name(uint16_t sw) {
    if (IN_STATE(sw, NOT_READY_TO_SWITCH_ON)) return "not ready to switch on";
    if (IN_STATE(sw, SWITCH_ON_DISABLED)) return "switch on disabled";
    if (IN_STATE(sw, READY_TO_SWITCH_ON)) return "ready to switch on";
    if (IN_STATE(sw, SWITCHED_ON)) return "switched on";
    if (IN_STATE(sw, OPERATION_ENABLED)) return "operation enabled";
    if (IN_STATE(sw, QUICK_STOP_ACTIVE)) return "quick stop active";
    if (IN_STATE(sw, FAULT_REACTION_ACTIVE)) return "fault reaction active";
    if (IN_STATE(sw, FAULT)) return "fault";
    return "unknown";
}

float sp_leg_actual_mm(const sp_core_t* c, uint8_t leg) {
    return (float)c->legs[leg].in.position / (float)SP_COUNTS_PER_MM;
}

bool sp_core_all_legs_in_csp(const sp_core_t* c) { return all_legs(c, M_CSP); }
