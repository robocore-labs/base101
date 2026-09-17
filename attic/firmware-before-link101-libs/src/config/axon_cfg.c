#include "axon_cfg.h"

#include <string.h>

#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

// Stored layout on flash: a small header + the config payload + CRC32.
#define AXON_CFG_MAGIC   0x41584F32u  // "AXO2"
#define AXON_CFG_VERSION 1u

// Dedicate one sector at a fixed 1 MiB offset. The program is well under
// 1 MiB and the RP2354B has 2 MiB of flash, so this sector is free at both
// ends. Keep it page-aligned and within FLASH_SECTOR_SIZE.
#define AXON_CFG_FLASH_OFFSET (1024u * 1024u)

typedef struct {
    uint32_t   magic;
    uint32_t   version;
    axon_cfg_t cfg;
    uint32_t   crc32;
} axon_cfg_blob_t;

_Static_assert(sizeof(axon_cfg_blob_t) <= FLASH_SECTOR_SIZE,
               "config blob must fit in one flash sector");

axon_cfg_t g_cfg;

static uint32_t crc32(const void *data, size_t len) {
    const uint8_t *p = (const uint8_t *)data;
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; i++) {
        crc ^= p[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1)));
        }
    }
    return ~crc;
}

void axon_cfg_defaults(void) {
    memset(&g_cfg, 0, sizeof(g_cfg));
    // Servos ON by default. (The node runs every configured servo regardless
    // of this flag anyway and tolerates offline ones; the flag is advisory.)
    g_cfg.servos_enabled = true;
    // Stock 6-DOF arm (IDs 1..6, joint names "1".."6"), so re-enabling the
    // subsystem reproduces the previous behaviour.
    g_cfg.servo_count = 6;
    for (uint8_t i = 0; i < g_cfg.servo_count; i++) {
        g_cfg.servos[i].id = (uint8_t)(i + 1);
        g_cfg.servos[i].enable = true;
        g_cfg.servos[i].joint[0] = (char)('1' + i);
        g_cfg.servos[i].joint[1] = '\0';
    }
}

bool axon_cfg_load(void) {
    const axon_cfg_blob_t *blob =
        (const axon_cfg_blob_t *)(XIP_BASE + AXON_CFG_FLASH_OFFSET);

    if (blob->magic == AXON_CFG_MAGIC && blob->version == AXON_CFG_VERSION &&
        blob->crc32 == crc32(&blob->cfg, sizeof(blob->cfg))) {
        memcpy(&g_cfg, &blob->cfg, sizeof(g_cfg));
        if (g_cfg.servo_count > AXON_SERVO_MAX) {
            g_cfg.servo_count = AXON_SERVO_MAX;  // guard against corruption
        }
        return true;
    }

    axon_cfg_defaults();
    return false;
}

bool axon_cfg_save(void) {
    // Assemble the blob in a page-aligned RAM buffer, then erase + program.
    static uint8_t page[FLASH_SECTOR_SIZE] __attribute__((aligned(4)));
    memset(page, 0xFF, sizeof(page));

    axon_cfg_blob_t *blob = (axon_cfg_blob_t *)page;
    blob->magic = AXON_CFG_MAGIC;
    blob->version = AXON_CFG_VERSION;
    memcpy(&blob->cfg, &g_cfg, sizeof(g_cfg));
    blob->crc32 = crc32(&blob->cfg, sizeof(blob->cfg));

    // Program a whole number of pages covering the blob.
    size_t prog_len = ((sizeof(axon_cfg_blob_t) + FLASH_PAGE_SIZE - 1) /
                       FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE;

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(AXON_CFG_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(AXON_CFG_FLASH_OFFSET, page, prog_len);
    restore_interrupts(ints);

    // Verify by reading back.
    const axon_cfg_blob_t *check =
        (const axon_cfg_blob_t *)(XIP_BASE + AXON_CFG_FLASH_OFFSET);
    return check->magic == AXON_CFG_MAGIC &&
           check->crc32 == crc32(&check->cfg, sizeof(check->cfg));
}
