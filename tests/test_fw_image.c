/* Firmware image slots and the bootloader decision, against the RAM flash model. */
#include "test.h"
#include <string.h>
#include "fw_image.h"
#include "crc.h"
#include "err.h"
#include "fake_flash.h"

#define SEC   4096U
#define SLOT  (2U * SEC)
#define MAXSZ (6U * SEC)

static uint8_t img[10000];

static void make_img(void)
{
    for (size_t i = 0; i < sizeof img; i++) { img[i] = (uint8_t)(i * 7U + 3U); }
}

static void fw_header_rules(void)
{
    fw_header_t h;
    fw_header_make(&h, FW_VERSION(1, 0, 1), 1000, 0x12345678UL);
    CHECK_EQ(fw_header_check(&h, MAXSZ), ERR_OK);
    CHECK_EQ(h.version, 0x010001);
    h.size = 1001;                                        /* any field changed: header CRC fails */
    CHECK_EQ(fw_header_check(&h, MAXSZ), ERR_CRC);
    fw_header_make(&h, 1, MAXSZ + 1, 0);
    CHECK_EQ(fw_header_check(&h, MAXSZ), ERR_PARAM);      /* too big for the slot */
    memset(&h, 0xFF, sizeof h);                           /* erased flash */
    CHECK_EQ(fw_header_check(&h, MAXSZ), ERR_NOT_FOUND);
}

static void fw_write_and_verify(void)
{
    make_img();
    fake_flash_t ff; flash_dev_t fd; fake_flash_init(&ff, &fd, 16 * SEC, SEC);
    fw_writer_t w;
    CHECK_EQ(fw_writer_begin(&w, &fd, SLOT, MAXSZ), ERR_OK);
    for (size_t off = 0; off < sizeof img; off += 700) {  /* odd chunk size, like UART packets */
        size_t n = sizeof img - off < 700 ? sizeof img - off : 700;
        CHECK_EQ(fw_writer_put(&w, &img[off], n), ERR_OK);
    }
    uint32_t crc = crc32(img, sizeof img);
    CHECK_EQ(fw_writer_finish(&w, FW_VERSION(1, 0, 1), crc), ERR_OK);
    fw_header_t h;
    CHECK_EQ(fw_slot_verify(&fd, SLOT, MAXSZ, &h), ERR_OK);
    CHECK_EQ(h.size, sizeof img);
    CHECK_EQ(h.version, FW_VERSION(1, 0, 1));
    CHECK(memcmp(&ff.mem[SLOT + FW_HDR_SPACE], img, sizeof img) == 0);
    CHECK_EQ(ff.mem[SLOT + 6 * SEC], 0xFF);               /* nothing written past the slot */

    /* one corrupted byte in flash: verify must fail */
    ff.mem[SLOT + FW_HDR_SPACE + 5000] ^= 0x01;
    CHECK_EQ(fw_slot_verify(&fd, SLOT, MAXSZ, NULL), ERR_CRC);
    fake_flash_free(&ff);
}

static void fw_bad_transfer(void)
{
    make_img();
    fake_flash_t ff; flash_dev_t fd; fake_flash_init(&ff, &fd, 16 * SEC, SEC);
    fw_writer_t w;
    /* a good image first */
    fw_writer_begin(&w, &fd, SLOT, MAXSZ);
    fw_writer_put(&w, img, sizeof img);
    CHECK_EQ(fw_writer_finish(&w, 1, crc32(img, sizeof img)), ERR_OK);

    /* new transfer with a wrong CRC: rejected, and the slot is empty (no half image) */
    CHECK_EQ(fw_writer_begin(&w, &fd, SLOT, MAXSZ), ERR_OK);
    fw_writer_put(&w, img, 4000);
    CHECK_EQ(fw_writer_finish(&w, 2, 0xDEADBEEFUL), ERR_CRC);
    CHECK_EQ(fw_slot_verify(&fd, SLOT, MAXSZ, NULL), ERR_NOT_FOUND);

    /* transfer cut halfway (power loss): still no valid image */
    CHECK_EQ(fw_writer_begin(&w, &fd, SLOT, MAXSZ), ERR_OK);
    fw_writer_put(&w, img, 3000);
    CHECK_EQ(fw_slot_verify(&fd, SLOT, MAXSZ, NULL), ERR_NOT_FOUND);

    /* image larger than the slot */
    static uint8_t big[MAXSZ + 1];
    CHECK_EQ(fw_writer_begin(&w, &fd, SLOT, MAXSZ), ERR_OK);
    CHECK_EQ(fw_writer_put(&w, big, sizeof big), ERR_FULL);
    CHECK_EQ(fw_writer_finish(&w, 3, 0), ERR_EMPTY);
    fake_flash_free(&ff);
}

static void fw_boot_flow(void)
{
    fw_boot_t st = { FW_STATE_IDLE, 0, FW_VERSION(1, 0, 0), 0 };
    CHECK_EQ(fw_boot_decide(&st, false, false, true, 3), FW_ACT_RUN);

    /* update requested, NEW slot valid: install, then trial */
    st.state = FW_STATE_PENDING;
    CHECK_EQ(fw_boot_decide(&st, true, false, true, 3), FW_ACT_INSTALL);
    CHECK_EQ(st.state, FW_STATE_TRIAL);
    CHECK_EQ(st.prev_version, FW_VERSION(1, 0, 0));
    st.version = FW_VERSION(1, 0, 1);

    /* new image confirms itself on the first boot */
    CHECK_EQ(fw_boot_decide(&st, true, true, true, 3), FW_ACT_RUN);
    CHECK(fw_boot_confirm(&st));
    CHECK_EQ(st.state, FW_STATE_IDLE);
    CHECK(!fw_boot_confirm(&st));                         /* second confirm does nothing */
    CHECK_EQ(fw_boot_decide(&st, true, true, true, 3), FW_ACT_RUN);
    CHECK_EQ(st.version, FW_VERSION(1, 0, 1));
}

static void fw_boot_rollback(void)
{
    fw_boot_t st = { FW_STATE_PENDING, 0, FW_VERSION(1, 0, 0), 0 };
    CHECK_EQ(fw_boot_decide(&st, true, false, true, 3), FW_ACT_INSTALL);
    st.version = FW_VERSION(1, 0, 2);                     /* this one hangs, never confirms */
    for (int i = 1; i <= 3; i++) {
        CHECK_EQ(fw_boot_decide(&st, true, true, true, 3), FW_ACT_RUN);
        CHECK_EQ(st.tries, (uint32_t)i);
    }
    CHECK_EQ(fw_boot_decide(&st, true, true, true, 3), FW_ACT_ROLLBACK);
    CHECK_EQ(st.state, FW_STATE_IDLE);
    CHECK_EQ(st.version, FW_VERSION(1, 0, 0));            /* back on the old version */

    /* trial without any backup: keep running the new image */
    fw_boot_t s2 = { FW_STATE_TRIAL, 3, 5, 0 };
    CHECK_EQ(fw_boot_decide(&s2, false, false, true, 3), FW_ACT_RUN);
    CHECK_EQ(s2.state, FW_STATE_IDLE);

    /* PENDING but the NEW slot got damaged: drop the request, run as before */
    fw_boot_t s3 = { FW_STATE_PENDING, 0, 7, 0 };
    CHECK_EQ(fw_boot_decide(&s3, false, true, true, 3), FW_ACT_RUN);
    CHECK_EQ(s3.state, FW_STATE_IDLE);

    /* internal application damaged, backup exists: restore it */
    fw_boot_t s4 = { FW_STATE_IDLE, 0, 9, 8 };
    CHECK_EQ(fw_boot_decide(&s4, false, true, false, 3), FW_ACT_ROLLBACK);
    CHECK_EQ(s4.version, 8);

    /* garbage state (erased flash): treated as IDLE */
    fw_boot_t s5 = { 0xFFFFFFFFUL, 0xFFFFFFFFUL, 0, 0 };
    CHECK_EQ(fw_boot_decide(&s5, false, false, true, 3), FW_ACT_RUN);
    CHECK_EQ(s5.state, FW_STATE_IDLE);
}

static void fw_vectors(void)
{
    const uint32_t A0 = 0x08008000UL, A1 = 0x08080000UL, R0 = 0x20000000UL, R1 = 0x20024000UL;
    CHECK(fw_vectors_ok(0x20024000UL, 0x0800A1C5UL, A0, A1, R0, R1));
    CHECK(!fw_vectors_ok(0xFFFFFFFFUL, 0xFFFFFFFFUL, A0, A1, R0, R1));   /* erased flash */
    CHECK(!fw_vectors_ok(0x20024000UL, 0x0800A1C4UL, A0, A1, R0, R1));  /* no Thumb bit */
    CHECK(!fw_vectors_ok(0x20024000UL, 0x08000101UL, A0, A1, R0, R1));  /* points into the bootloader */
    CHECK(!fw_vectors_ok(0x20030000UL, 0x0800A1C5UL, A0, A1, R0, R1));  /* SP outside RAM */
}

void suite_fw_image(void)
{
    RUN(fw_header_rules);
    RUN(fw_write_and_verify);
    RUN(fw_bad_transfer);
    RUN(fw_boot_flow);
    RUN(fw_boot_rollback);
    RUN(fw_vectors);
}
