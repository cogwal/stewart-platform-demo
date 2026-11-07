#include "sp_co_adapter.h"

#include <string.h>

#include "OD.h"

static sp_co_t* g_ad; /* CANopenNode's EMCY callback has no context pointer */

static const uint16_t TPDO_BASES[SP_N_DRIVE_TPDO] = SP_DRIVE_TPDO_BASES;
static const uint16_t RPDO_BASES[SP_N_DRIVE_RPDO] = SP_DRIVE_RPDO_BASES;

void sp_co_configure_od(const uint8_t* nodes, uint8_t n_legs, uint8_t master_id) {
    (void)master_id;
    for (uint8_t leg = 0; leg < SP_MAX_LEGS; leg++) {
        bool used = leg < n_legs && nodes[leg] != 0;
        uint8_t node = used ? nodes[leg] : (uint8_t)(leg + 1);
        OD_set_u32(OD_find(OD, 0x1016), (uint8_t)(leg + 1),
                   used ? ((uint32_t)node << 16) | SP_HB_MASTER_CONSUMES_DRIVE_MS : 0u, true);
        for (uint8_t j = 0; j < SP_N_DRIVE_TPDO; j++) { /* master RPDOs <- drive TPDOs */
            uint32_t cob = (uint32_t)TPDO_BASES[j] + node;
            OD_set_u32(OD_find(OD, (uint16_t)(0x1400 + leg * SP_N_DRIVE_TPDO + j)), 1, used ? cob : (0x80000000u | cob),
                       true);
        }
        for (uint8_t j = 0; j < SP_N_DRIVE_RPDO; j++) { /* master TPDOs -> drive RPDOs */
            uint32_t cob = 0x40000000u | ((uint32_t)RPDO_BASES[j] + node);
            OD_set_u32(OD_find(OD, (uint16_t)(0x1800 + leg * SP_N_DRIVE_RPDO + j)), 1, used ? cob : (0x80000000u | cob),
                       true);
        }
    }
}

/* ---- sp_port_t -------------------------------------------------------------------------- */
static void p_nmt(void* ctx, uint8_t cmd, uint8_t node) {
    sp_co_t* ad = ctx;
    (void)CO_NMT_sendCommand(ad->co->NMT, (CO_NMT_command_t)cmd, node);
}

static bool sdo_setup(sp_co_t* ad, uint8_t node) {
    if (ad->sdo_active) {
        return false;
    }
    return CO_SDOclient_setup(ad->co->SDOclient, CO_CAN_ID_SDO_CLI + node, CO_CAN_ID_SDO_SRV + node, node) ==
           CO_SDO_RT_ok_communicationEnd;
}

static bool p_sdo_write(void* ctx, uint8_t node, uint16_t idx, uint8_t sub, uint32_t value, uint8_t size) {
    sp_co_t* ad = ctx;
    uint8_t buf[4] = {(uint8_t)value, (uint8_t)(value >> 8), (uint8_t)(value >> 16), (uint8_t)(value >> 24)};
    if (size == 0 || size > 4 || !sdo_setup(ad, node)) {
        return false;
    }
    if (CO_SDOclientDownloadInitiate(ad->co->SDOclient, idx, sub, size, SP_TIMEOUT_SDO_MS, false) !=
        CO_SDO_RT_ok_communicationEnd) {
        return false;
    }
    CO_SDOclientDownloadBufWrite(ad->co->SDOclient, buf, size);
    ad->sdo_active = true;
    ad->sdo_write = true;
    ad->sdo_size = size;
    return true;
}

static bool p_sdo_read(void* ctx, uint8_t node, uint16_t idx, uint8_t sub) {
    sp_co_t* ad = ctx;
    if (!sdo_setup(ad, node)) {
        return false;
    }
    if (CO_SDOclientUploadInitiate(ad->co->SDOclient, idx, sub, SP_TIMEOUT_SDO_MS, false) !=
        CO_SDO_RT_ok_communicationEnd) {
        return false;
    }
    ad->sdo_active = true;
    ad->sdo_write = false;
    return true;
}

static void send_tpdo(sp_co_t* ad, uint16_t k) {
    if (k < OD_CNT_TPDO) {
        CO_TPDOsendRequest(&ad->co->TPDO[k]);
    }
}

static void p_rpdo1(void* ctx, uint8_t leg, uint16_t cw, int32_t pos) {
    sp_co_t* ad = ctx;
    OD_RAM.x2120_driveControlword[leg] = cw;
    OD_RAM.x2121_driveTargetPosition[leg] = pos;
    send_tpdo(ad, (uint16_t)(leg * SP_N_DRIVE_RPDO + SP_MASTER_TPDO_OFFSET_RPDO1));
}

static void p_rpdo2(void* ctx, uint8_t leg, uint16_t cw, int8_t mode) {
    sp_co_t* ad = ctx;
    OD_RAM.x2120_driveControlword[leg] = cw;
    OD_RAM.x2122_driveModeOfOperation[leg] = mode;
    send_tpdo(ad, (uint16_t)(leg * SP_N_DRIVE_RPDO + SP_MASTER_TPDO_OFFSET_RPDO2));
}

static void p_rpdo3(void* ctx, uint8_t leg, uint16_t cw, int32_t vel) {
    sp_co_t* ad = ctx;
    OD_RAM.x2120_driveControlword[leg] = cw;
    OD_RAM.x2123_driveTargetVelocity[leg] = vel;
    send_tpdo(ad, (uint16_t)(leg * SP_N_DRIVE_RPDO + SP_MASTER_TPDO_OFFSET_RPDO3));
}

void sp_co_port(sp_co_t* ad, sp_port_t* port, void (*log)(void* ctx, int level, const char* text)) {
    memset(port, 0, sizeof *port);
    port->ctx = ad;
    port->nmt = p_nmt;
    port->sdo_write = p_sdo_write;
    port->sdo_read = p_sdo_read;
    port->rpdo1 = p_rpdo1;
    port->rpdo2 = p_rpdo2;
    port->rpdo3 = p_rpdo3;
    port->log = log;
}

/* ---- callbacks -------------------------------------------------------------------------- */
static void emcy_rx(const uint16_t ident, const uint16_t code, const uint8_t reg, const uint8_t byte1,
                    const uint32_t info) {
    (void)info;
    if (g_ad != NULL && ident != 0) {
        sp_core_on_emcy(&g_ad->app->core, (uint8_t)(ident & 0x7F), code, reg, byte1);
    }
}

static void hb_remote_reset(uint8_t node, uint8_t idx, void* object) {
    sp_co_t* ad = object;
    (void)idx;
    sp_core_on_bootup(&ad->app->core, node);
}

void sp_co_init(sp_co_t* ad, sp_app_t* app, const uint8_t* nodes, uint8_t n_legs) {
    memset(ad, 0, sizeof *ad);
    ad->app = app;
    ad->n_legs = n_legs;
    memcpy(ad->nodes, nodes, SP_MAX_LEGS);
    g_ad = ad;
}

void sp_co_on_comm_reset(sp_co_t* ad, CO_t* co) {
    ad->co = co;
    ad->sdo_active = false;
    CO_EM_initCallbackRx(co->em, emcy_rx);
    for (uint8_t i = 0; i < ad->n_legs; i++) {
        CO_HBconsumer_initCallbackRemoteReset(co->HBcons, i, ad, hb_remote_reset);
    }
    if (ad->app->core.sdo_busy) {
        sp_core_sdo_done(&ad->app->core, 0x08000000u, 0); /* transfer lost with the reset */
    }
}

static void sdo_process(sp_co_t* ad, uint32_t dt_us) {
    CO_SDO_abortCode_t abort_code = CO_SDO_AB_NONE;
    CO_SDO_return_t ret;
    if (!ad->sdo_active) {
        return;
    }
    if (ad->sdo_write) {
        ret = CO_SDOclientDownload(ad->co->SDOclient, dt_us, false, false, &abort_code, NULL, NULL);
    } else {
        ret = CO_SDOclientUpload(ad->co->SDOclient, dt_us, false, &abort_code, NULL, NULL, NULL);
    }
    if (ret > 0) {
        return; /* in progress */
    }
    ad->sdo_active = false;
    uint32_t value = 0;
    if (ret == CO_SDO_RT_ok_communicationEnd && !ad->sdo_write) {
        uint8_t buf[4] = {0};
        size_t n = CO_SDOclientUploadBufRead(ad->co->SDOclient, buf, sizeof buf);
        for (size_t k = 0; k < n && k < 4; k++) {
            value |= (uint32_t)buf[k] << (8 * k);
        }
    }
    sp_core_sdo_done(&ad->app->core, ret == CO_SDO_RT_ok_communicationEnd ? 0u : (uint32_t)abort_code, value);
}

void sp_co_process(sp_co_t* ad, uint32_t dt_us) {
    if (ad->co == NULL) {
        return;
    }
    sdo_process(ad, dt_us);
    for (uint8_t i = 0; i < ad->n_legs; i++) {
        sp_leg_in_t* in = sp_core_leg_in(&ad->app->core, i);
        CO_NMT_internalState_t nmt;
        in->hb_active = CO_HBconsumer_getState(ad->co->HBcons, i) == CO_HBconsumer_ACTIVE;
        in->nmt = CO_HBconsumer_getNmtState(ad->co->HBcons, i, &nmt) == 0 ? (int16_t)nmt : (int16_t)SP_NMTS_UNKNOWN;
        in->statusword = OD_RAM.x2110_driveStatusword[i];
        in->position = OD_RAM.x2111_drivePositionActual[i];
        in->mode_display = OD_RAM.x2112_driveModeDisplay[i];
    }
    sp_app_step(ad->app, dt_us);
    /* SYNC just produced: the producer's timer restarted. */
    uint32_t timer = ad->co->SYNC->timer;
    if (timer < ad->sync_timer_prev) {
        ad->syncs++;
        sp_app_on_sync(ad->app);
    }
    ad->sync_timer_prev = timer;
}
