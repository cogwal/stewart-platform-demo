#include "fake_bus.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const uint16_t SW[] = {0x0040, 0x0021, 0x0023, 0x0027, 0x0007, 0x0008};

fdrive_t* fake_drive_by_node(fake_t* f, uint8_t node) {
    for (uint8_t i = 0; i < f->n; i++) {
        if (f->d[i].node == node) {
            return &f->d[i];
        }
    }
    return NULL;
}

static uint16_t statusword(const fdrive_t* d) {
    uint16_t sw = SW[d->state] | 0x0200; /* bit 9 remote */
    if (d->state == FD_OE) {
        if (d->mode == SP_MODE_HOMING && d->homing_attained) {
            sw |= 0x1400;
        } else if (d->mode == SP_MODE_PROFILE_POSITION) {
            if (d->pp_ack) {
                sw |= 0x1000;
            }
            if (!d->pp_moving) {
                sw |= 0x0400;
            }
        } else if (d->mode == SP_MODE_CYCLIC_SYNC_POSITION) {
            sw |= 0x1000;
        }
    }
    return sw;
}

static void fault(fdrive_t* d, int emcy) {
    d->state = FD_FAULT;
    d->vel_pps = 0;
    d->pp_moving = false;
    if (emcy) {
        d->emcy_pending = emcy;
    }
}

static void controlword(fdrive_t* d, uint16_t cw) {
    uint16_t prev = d->cw;
    d->cw = cw;
    switch (d->state) {
        case FD_FAULT:
            if ((cw & 0x80) && !(prev & 0x80)) {
                d->state = FD_SOD;
            }
            break;
        case FD_SOD:
            if ((cw & 0x87) == 0x06) d->state = FD_RTSO;
            break;
        case FD_RTSO:
            if ((cw & 0x8F) == 0x07) d->state = FD_SO;
            else if (!(cw & 0x02) || !(cw & 0x04)) d->state = FD_SOD;
            break;
        case FD_SO:
            if ((cw & 0x8F) == 0x0F) d->state = FD_OE;
            else if ((cw & 0x87) == 0x06) d->state = FD_RTSO;
            else if (!(cw & 0x02) || !(cw & 0x04)) d->state = FD_SOD;
            break;
        case FD_OE:
            if ((cw & 0x87) == 0x06) d->state = FD_RTSO;
            else if ((cw & 0x8F) == 0x07) d->state = FD_SO;
            else if ((cw & 0x86) == 0x02) { d->state = FD_SOD; d->vel_pps = 0; d->pp_moving = false; } /* quick stop, option 2 */
            else if (!(cw & 0x02)) d->state = FD_SOD;
            break;
        default:
            break;
    }
    if (d->state != FD_OE) {
        return;
    }
    bool rise4 = (cw & 0x10) && !(prev & 0x10);
    if (d->mode == SP_MODE_HOMING && rise4) {
        d->counts = 0; /* method 35: here is home */
        d->homing_attained = true;
    } else if (d->mode == SP_MODE_PROFILE_POSITION) {
        if (rise4) {
            d->pp_ack = true;
            d->pp_moving = true;
        } else if (!(cw & 0x10)) {
            d->pp_ack = false;
        }
    }
}

static void set_mode(fdrive_t* d, int8_t mode) {
    if (mode != d->mode) {
        d->mode = mode;
        d->homing_attained = false;
        d->pp_ack = false;
        d->pp_moving = false;
        d->vel_pps = 0;
    }
}

/* ---- port ------------------------------------------------------------------------------- */
static void p_nmt(void* ctx, uint8_t cmd, uint8_t node) {
    fake_t* f = ctx;
    fdrive_t* d = fake_drive_by_node(f, node);
    if (d == NULL || !d->powered) {
        return;
    }
    if (cmd == SP_NMT_RESET_NODE || cmd == SP_NMT_RESET_COMM) {
        d->state = FD_SOD;
        d->mode = 0;
        d->nmt = SP_NMTS_PREOP;
        d->bootup_pending = true;
        d->vel_pps = 0;
        d->counts = 0;
    } else if (cmd == SP_NMT_START) {
        d->nmt = SP_NMTS_OPERATIONAL;
    } else if (cmd == SP_NMT_STOP) {
        d->nmt = SP_NMTS_STOPPED;
        fault(d, 0);
    } else if (cmd == SP_NMT_PREOP) {
        d->nmt = SP_NMTS_PREOP;
    }
}

static bool p_sdo(fake_t* f, bool write, uint8_t node, uint16_t idx, uint8_t sub, uint32_t value) {
    if (f->sdo_active) {
        return false;
    }
    f->sdo_active = true;
    f->sdo_write = write;
    f->sdo_node = node;
    f->sdo_index = idx;
    f->sdo_sub = sub;
    f->sdo_value = value;
    return true;
}

static bool p_sdo_write(void* ctx, uint8_t node, uint16_t idx, uint8_t sub, uint32_t value, uint8_t size) {
    (void)size;
    return p_sdo(ctx, true, node, idx, sub, value);
}

static bool p_sdo_read(void* ctx, uint8_t node, uint16_t idx, uint8_t sub) { return p_sdo(ctx, false, node, idx, sub, 0); }

static fdrive_t* leg_drive(fake_t* f, uint8_t leg) {
    fdrive_t* d = leg < f->n ? &f->d[leg] : NULL;
    return (d && d->powered && d->nmt == SP_NMTS_OPERATIONAL) ? d : NULL; /* PDOs only in OPERATIONAL */
}

static void p_rpdo1(void* ctx, uint8_t leg, uint16_t cw, int32_t pos) {
    fake_t* f = ctx;
    fdrive_t* d = leg_drive(f, leg);
    f->frames++;
    if (d == NULL) return;
    if (d->mode == SP_MODE_PROFILE_POSITION) d->pp_target = pos;
    if (d->mode == SP_MODE_CYCLIC_SYNC_POSITION && d->state == FD_OE) {
        double delta = pos - d->counts;
        d->counts = pos;
        d->truth_mm += delta / SP_COUNTS_PER_MM;
    }
    controlword(d, cw);
}

static void p_rpdo2(void* ctx, uint8_t leg, uint16_t cw, int8_t mode) {
    fake_t* f = ctx;
    fdrive_t* d = leg_drive(f, leg);
    f->frames++;
    if (d == NULL) return;
    set_mode(d, mode);
    controlword(d, cw);
}

static void p_rpdo3(void* ctx, uint8_t leg, uint16_t cw, int32_t vel) {
    fake_t* f = ctx;
    fdrive_t* d = leg_drive(f, leg);
    f->frames++;
    if (d == NULL) return;
    d->pv_target = vel;
    controlword(d, cw);
}

static void p_log(void* ctx, int level, const char* text) {
    fake_t* f = ctx;
    if (level <= f->log_level) {
        printf("    [%6u ms] %s\n", f->t_ms, text);
    }
}

/* ---- simulation ------------------------------------------------------------------------- */
static void drive_tick(fdrive_t* d, double dt) {
    if (!d->powered || d->state != FD_OE) {
        d->vel_pps = 0;
        return;
    }
    double v = 0;
    if (d->mode == SP_MODE_PROFILE_VELOCITY) {
        v = d->pv_target;
    } else if (d->mode == SP_MODE_PROFILE_POSITION && d->pp_moving) {
        double step = SP_PROFILE_VELOCITY_MM_S * SP_COUNTS_PER_MM * dt;
        double err = d->pp_target - d->counts;
        if (fabs(err) <= step) {
            d->truth_mm += err / SP_COUNTS_PER_MM;
            d->counts = d->pp_target;
            d->pp_moving = false;
            return;
        }
        v = err > 0 ? SP_PROFILE_VELOCITY_MM_S * SP_COUNTS_PER_MM : -SP_PROFILE_VELOCITY_MM_S * SP_COUNTS_PER_MM;
    }
    d->vel_pps = v;
    d->counts += v * dt;
    d->truth_mm += v * dt / SP_COUNTS_PER_MM;
    d->stalled = false;
    if (d->truth_mm < 0) {
        d->truth_mm = 0;
        d->stalled = v < 0;
    } else if (d->truth_mm > SP_STROKE_MM) {
        d->truth_mm = SP_STROKE_MM;
        d->stalled = v > 0;
    }
    if (d->stalled && !d->no_stall_detect) {
        if (d->fault_on_stall) {
            fault(d, 0x3230);
        } else {
            d->emcy_pending = 0x3230;
            d->vel_pps = 0;
            d->pv_target = 0;
        }
    }
}

static void sdo_complete(fake_t* f) {
    fdrive_t* d = fake_drive_by_node(f, f->sdo_node);
    uint32_t value = 0, abort_code = 0;
    if (!f->sdo_active || f->sdo_hang) {
        return;
    }
    f->sdo_active = false;
    if (d == NULL || !d->powered) {
        sp_core_sdo_done(&f->app.core, 0x05040000u, 0);
        return;
    }
    if (f->sdo_write) {
        d->sdo_writes++;
        if (f->sdo_index == 0x6040) controlword(d, (uint16_t)f->sdo_value);
        if (f->sdo_index == 0x2003) d->current = (uint8_t)f->sdo_value;
    } else {
        switch (f->sdo_index) {
            case 0x1018: value = f->sdo_sub == 1 ? 0x286u : f->sdo_sub == 2 ? 1270u : 0x0003001Bu; break;
            case 0x6041: value = statusword(d); break;
            case 0x2101: value = (d->stalled || (d->no_stall_detect && d->truth_mm <= 0 && d->vel_pps < 0)) ? 0 : 600; break;
            default: abort_code = 0x06020000u; break;
        }
    }
    sp_core_sdo_done(&f->app.core, abort_code, value);
}

void fake_init(fake_t* f, uint8_t n, double rod_mm, bool auto_home) {
    static const uint8_t nodes[SP_MAX_LEGS] = SP_DRIVE_NODE_IDS;
    sp_port_t port = {f, p_nmt, p_sdo_write, p_sdo_read, p_rpdo1, p_rpdo2, p_rpdo3, p_log};
    memset(f, 0, sizeof *f);
    f->n = n;
    f->log_level = getenv("FAKE_LOG") ? atoi(getenv("FAKE_LOG")) : -1;
    for (uint8_t i = 0; i < n; i++) {
        fdrive_t* d = &f->d[i];
        d->node = nodes[i];
        d->powered = true;
        d->hb_alive = true;
        d->nmt = SP_NMTS_PREOP;
        d->state = FD_SOD;
        d->truth_mm = rod_mm + 3.0 * i;
        d->fault_on_stall = true;
    }
    sp_app_init(&f->app, &port, nodes, n, SP_MASTER_NODE_ID, auto_home);
}

void fake_power_cycle(fake_t* f, uint8_t leg) {
    fdrive_t* d = &f->d[leg];
    d->state = FD_SOD;
    d->mode = 0;
    d->counts = 0;
    d->vel_pps = 0;
    d->nmt = SP_NMTS_PREOP;
    d->bootup_pending = true;
}

static void tick(fake_t* f) {
    f->t_ms++;
    sdo_complete(f);
    for (uint8_t i = 0; i < f->n; i++) {
        fdrive_t* d = &f->d[i];
        drive_tick(d, 0.001);
        sp_leg_in_t* in = sp_core_leg_in(&f->app.core, i);
        in->hb_active = d->powered && d->hb_alive;
        in->nmt = (int16_t)(in->hb_active ? d->nmt : SP_NMTS_UNKNOWN);
        if (d->nmt == SP_NMTS_OPERATIONAL) { /* TPDOs */
            in->statusword = statusword(d);
            in->position = (int32_t)lround(d->counts);
            in->mode_display = d->mode;
        }
        if (d->bootup_pending && d->powered) {
            d->bootup_pending = false;
            sp_core_on_bootup(&f->app.core, d->node);
        }
        if (d->emcy_pending) {
            sp_core_on_emcy(&f->app.core, d->node, (uint16_t)d->emcy_pending, 0x80, 0);
            d->emcy_pending = 0;
        }
    }
    sp_app_step(&f->app, 1000);
    if (f->t_ms % SP_SYNC_PERIOD_MS == 0) {
        sp_app_on_sync(&f->app);
    }
}

void fake_run(fake_t* f, uint32_t ms) {
    for (uint32_t k = 0; k < ms; k++) {
        tick(f);
    }
}

bool fake_run_until(fake_t* f, uint32_t max_ms, bool (*cond)(fake_t*)) {
    for (uint32_t k = 0; k < max_ms; k++) {
        if (cond(f)) {
            return true;
        }
        tick(f);
    }
    return cond(f);
}
