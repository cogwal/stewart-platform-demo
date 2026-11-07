/* CANopenNode adapter for the master core: implements sp_port_t with the CANopenNode v4 objects
 * (NMT master, SDO client, HB consumer, EMCY consumer, SYNC producer, PDOs mapped to the generated
 * master OD). Platform independent: used by CANopenLinux (master/linux) and later the TTC2000 ECU.
 *
 * Integration (any CANopenNode main loop, single thread):
 *   after storage load, before CO_CANopenInit:  sp_co_configure_od(nodes, n, master_id)
 *   after CO_CANopenInit (communication reset): sp_co_on_comm_reset(ad, CO)
 *   every loop, after CO_process / RPDO / TPDO: sp_co_process(ad, dt_us)
 */
#ifndef SP_CO_ADAPTER_H
#define SP_CO_ADAPTER_H

#include "CANopen.h"
#include "sp_app.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    CO_t* co;
    sp_app_t* app;
    uint8_t nodes[SP_MAX_LEGS];
    uint8_t n_legs;
    /* SDO in progress */
    bool sdo_active, sdo_write;
    uint8_t sdo_size;
    /* SYNC detection */
    uint32_t sync_timer_prev;
    uint32_t syncs;
} sp_co_t;

/* Patch the OD before CANopenInit: heartbeat consumer entries and PDO COB-IDs for the drive node ids
 * actually used (the generated defaults are for contract nodes 1..6). */
void sp_co_configure_od(const uint8_t* nodes, uint8_t n_legs, uint8_t master_id);
/* Fill an sp_port_t that talks through this adapter. */
void sp_co_port(sp_co_t* ad, sp_port_t* port, void (*log)(void* ctx, int level, const char* text));
void sp_co_init(sp_co_t* ad, sp_app_t* app, const uint8_t* nodes, uint8_t n_legs);
void sp_co_on_comm_reset(sp_co_t* ad, CO_t* co);
void sp_co_process(sp_co_t* ad, uint32_t dt_us);

#ifdef __cplusplus
}
#endif

#endif /* SP_CO_ADAPTER_H */
