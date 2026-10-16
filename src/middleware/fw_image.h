/**
 * @file    fw_image.h
 * @brief   Firmware images on external flash and the update decision logic.
 *
 * Shared by the bootloader and the application, with no HAL inside, so the
 * whole update logic is unit-tested on the PC.
 *
 * A slot on the external NOR flash holds one image:
 *   slot + 0                : fw_header_t (written LAST, after the CRC check)
 *   slot + FW_HDR_SPACE     : the raw application binary
 * A slot without a valid header is "empty", so a transfer cut halfway never
 * looks like a valid image.
 *
 * The update state (fw_boot_t) is kept with config_store (A/B copies), so a
 * power cut while it is saved keeps the previous state.
 */
#ifndef FW_IMAGE_H
#define FW_IMAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "bus.h"

#define FW_MAGIC        0x31495746UL      /* "FWI1" */
#define FW_HDR_SPACE    256U              /* image data starts here in a slot */
#define FW_VERSION(maj, min, pat) (((uint32_t)(maj) << 16) | ((uint32_t)(min) << 8) | (uint32_t)(pat))

typedef struct {
    uint32_t magic;
    uint32_t version;       /* FW_VERSION(major, minor, patch) */
    uint32_t size;          /* bytes of the application binary */
    uint32_t crc;           /* CRC-32 of the binary */
    uint32_t hdr_crc;       /* CRC-32 of the four fields above */
} fw_header_t;

void fw_header_make(fw_header_t *h, uint32_t version, uint32_t size, uint32_t crc);
/** @return ERR_OK, ERR_NOT_FOUND (no magic: empty slot), ERR_CRC or ERR_PARAM (size). */
int  fw_header_check(const fw_header_t *h, uint32_t max_size);
/** Read the header of a slot and check the CRC of the whole image. */
int  fw_slot_verify(const flash_dev_t *f, uint32_t slot, uint32_t max_size, fw_header_t *out);

/* ---- Writing an image into a slot, piece by piece (data arrives over UART) ---- */
typedef struct {
    const flash_dev_t *flash;
    uint32_t slot, max_size;
    uint32_t written;       /* image bytes stored so far */
    uint32_t crc;           /* running CRC-32 of the image */
    uint32_t erased_to;     /* first address not erased yet */
} fw_writer_t;

/** Erase the header sector first: the old image is invalid from now on. */
int fw_writer_begin(fw_writer_t *w, const flash_dev_t *f, uint32_t slot, uint32_t max_size);
int fw_writer_put(fw_writer_t *w, const void *data, size_t len);
/** Check the whole-image CRC, then write the header. */
int fw_writer_finish(fw_writer_t *w, uint32_t version, uint32_t expect_crc);

/* ---- What the bootloader does at reset ---- */
typedef enum {
    FW_STATE_IDLE    = 0,   /* running a confirmed image                    */
    FW_STATE_PENDING = 1,   /* a verified image waits in the NEW slot       */
    FW_STATE_TRIAL   = 2,   /* new image installed, not confirmed yet       */
} fw_state_t;

typedef struct {
    uint32_t state;         /* fw_state_t */
    uint32_t tries;         /* boots of the TRIAL image without a confirm */
    uint32_t version;       /* version running (or being tried) */
    uint32_t prev_version;  /* version kept in the BACKUP slot */
} fw_boot_t;

typedef enum {
    FW_ACT_RUN = 0,         /* jump to the application                       */
    FW_ACT_INSTALL,         /* back up the application, copy NEW into flash  */
    FW_ACT_ROLLBACK,        /* copy BACKUP into flash                        */
} fw_action_t;

/**
 * Decide what to do at this reset and update the state to save.
 * @param new_ok     the NEW slot holds a valid image
 * @param backup_ok  the BACKUP slot holds a valid image
 * @param app_ok     the internal flash holds a plausible application
 * @param max_tries  boots allowed to a TRIAL image before rolling back
 */
fw_action_t fw_boot_decide(fw_boot_t *st, bool new_ok, bool backup_ok, bool app_ok, uint32_t max_tries);
/** Called by the application once it runs well: TRIAL -> IDLE. @return true if changed. */
bool fw_boot_confirm(fw_boot_t *st);

/** Sanity check of an application vector table (initial SP and reset handler). */
bool fw_vectors_ok(uint32_t sp, uint32_t reset, uint32_t app_start, uint32_t app_end,
                   uint32_t ram_start, uint32_t ram_end);

#endif /* FW_IMAGE_H */
