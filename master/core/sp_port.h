/* The master core's view of the bus. Implemented by an adapter per platform:
 *   master/canopennode/sp_co_adapter.c  CANopenNode (PC with CANopenLinux, and the TTC2000 ECU)
 *   master/tests/fake_bus.c             unit tests (fake drives, fake time)
 *
 * The core never blocks: requests return immediately, results come back through sp_core_*()
 * input functions (see sp_core.h). All calls happen from one thread / task.
 */
#ifndef SP_PORT_H
#define SP_PORT_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NMT commands (CiA 301). */
enum { SP_NMT_START = 0x01, SP_NMT_STOP = 0x02, SP_NMT_PREOP = 0x80, SP_NMT_RESET_NODE = 0x81, SP_NMT_RESET_COMM = 0x82 };

typedef struct sp_port {
    void* ctx;
    /* NMT command to one drive (node id). */
    void (*nmt)(void* ctx, uint8_t command, uint8_t node);
    /* Start an expedited SDO transfer. Only one is outstanding at a time; the adapter reports the
     * result with sp_core_sdo_done(). Returns false if it could not be started. */
    bool (*sdo_write)(void* ctx, uint8_t node, uint16_t index, uint8_t sub, uint32_t value, uint8_t size);
    bool (*sdo_read)(void* ctx, uint8_t node, uint16_t index, uint8_t sub);
    /* Drive RPDOs (contract: RPDO1 controlword + target position, RPDO2 controlword + mode,
     * RPDO3 controlword + target velocity). leg = 0..SP_MAX_LEGS-1. */
    void (*rpdo1)(void* ctx, uint8_t leg, uint16_t controlword, int32_t target_position);
    void (*rpdo2)(void* ctx, uint8_t leg, uint16_t controlword, int8_t mode);
    void (*rpdo3)(void* ctx, uint8_t leg, uint16_t controlword, int32_t target_velocity);
    /* Optional log sink (may be NULL). level: 0 error, 1 warning, 2 info. */
    void (*log)(void* ctx, int level, const char* text);
} sp_port_t;

#ifdef __cplusplus
}
#endif

#endif /* SP_PORT_H */
