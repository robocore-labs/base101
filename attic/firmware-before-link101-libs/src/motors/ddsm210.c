#include "ddsm210.h"

#include <string.h>

#include "pico/stdlib.h"
#include "tusb.h"

#define DDSM_FRAME_SIZE 10
// The reference python driver reads with a 100 ms ceiling; a motor can be
// slower to answer than the old 4 ms window allowed. 8 ms keeps the control
// loop responsive (a present motor still replies in ~1-2 ms and returns early)
// while giving discovery enough margin to actually see replies.
#define DDSM_RESPONSE_TIMEOUT_US 8000
#define DDSM_RETRIES 2

// CRC-8/MAXIM lookup table (poly 0x31 reflected), same as the python driver.
static const uint8_t crc8_table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E,
    0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0,
    0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D,
    0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5,
    0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58,
    0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6,
    0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B,
    0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F,
    0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92,
    0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C,
    0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1,
    0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49,
    0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4,
    0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A,
    0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7,
    0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

static uint8_t crc8_maxim(const uint8_t *data, uint32_t len) {
    uint8_t crc = 0x00;
    for (uint32_t i = 0; i < len; i++) {
        crc = crc8_table[crc ^ data[i]];
    }
    return crc;
}

static void build_frame(uint8_t *frame, const uint8_t *data9) {
    memcpy(frame, data9, 9);
    frame[9] = crc8_maxim(frame, 9);
}

// Keep USB and the UART rings serviced while waiting on the motor.
// Safe from both main-loop and zenoh-idle context: never touches zenoh.
static void ddsm_poll(void) {
    tud_task();
    ddsm_port_task();
}

static void flush_rx(ddsm_port_id_t port) {
    uint8_t scratch[32];
    ddsm_port_task();
    while (ddsm_port_read(port, scratch, sizeof(scratch)) > 0) {
    }
}

// Send a frame and wait for the 10-byte response. Returns true and fills
// resp on success (CRC checked), retries on timeout/corruption.
static bool transact(ddsm_port_id_t port, const uint8_t *frame, uint8_t *resp) {
    for (int attempt = 0; attempt <= DDSM_RETRIES; attempt++) {
        flush_rx(port);
        ddsm_port_write(port, frame, DDSM_FRAME_SIZE);

        absolute_time_t deadline = make_timeout_time_us(DDSM_RESPONSE_TIMEOUT_US);
        uint32_t got = 0;
        while (got < DDSM_FRAME_SIZE && !time_reached(deadline)) {
            ddsm_poll();
            got += ddsm_port_read(port, &resp[got], DDSM_FRAME_SIZE - got);
        }

        if (got == DDSM_FRAME_SIZE && crc8_maxim(resp, 9) == resp[9]) return true;
    }
    return false;
}

static void send_no_response(ddsm_port_id_t port, const uint8_t *frame) {
    ddsm_port_write(port, frame, DDSM_FRAME_SIZE);
}

// -- Protocol 1: drive (0x64) -----------------------------------------------

static bool drive(ddsm_port_id_t port, uint8_t motor_id, uint16_t value,
                  uint8_t accel_time, bool brake, ddsm210_feedback_t *fb) {
    uint8_t data[9] = {
        motor_id, 0x64,
        (uint8_t)(value >> 8), (uint8_t)(value & 0xFF),
        0x01,                      // feedback slot 1: speed
        0x03,                      // feedback slot 2: position
        accel_time,
        brake ? 0xFF : 0x00,
        0x00,
    };
    uint8_t frame[DDSM_FRAME_SIZE];
    uint8_t resp[DDSM_FRAME_SIZE];
    build_frame(frame, data);

    if (!transact(port, frame, resp)) {
        return false;
    }
    if (fb != NULL) {
        fb->feedback1 = (int16_t)(((uint16_t)resp[2] << 8) | resp[3]);
        fb->feedback2 = (int16_t)(((uint16_t)resp[4] << 8) | resp[5]);
        fb->accel_time = resp[6];
        fb->temperature = resp[7];
        fb->error_code = resp[8];
    }
    return true;
}

bool ddsm210_set_velocity(ddsm_port_id_t port, uint8_t motor_id, int16_t rpm_x10,
                          uint8_t accel_time, ddsm210_feedback_t *fb) {
    if (rpm_x10 > DDSM210_MAX_SPEED_RAW) rpm_x10 = DDSM210_MAX_SPEED_RAW;
    if (rpm_x10 < -DDSM210_MAX_SPEED_RAW) rpm_x10 = -DDSM210_MAX_SPEED_RAW;
    return drive(port, motor_id, (uint16_t)rpm_x10, accel_time, false, fb);
}

bool ddsm210_brake(ddsm_port_id_t port, uint8_t motor_id) {
    return drive(port, motor_id, 0, 0, true, NULL);
}

bool ddsm210_set_position(ddsm_port_id_t port, uint8_t motor_id, uint16_t position,
                          ddsm210_feedback_t *fb) {
    if (position > DDSM210_MAX_POSITION) position = DDSM210_MAX_POSITION;
    return drive(port, motor_id, position, 0, false, fb);
}

// -- Protocol 2: odometry (0x74) --------------------------------------------

bool ddsm210_get_odometry(ddsm_port_id_t port, uint8_t motor_id, ddsm210_odometry_t *odom) {
    uint8_t data[9] = {motor_id, 0x74, 0, 0, 0, 0, 0, 0, 0};
    uint8_t frame[DDSM_FRAME_SIZE];
    uint8_t resp[DDSM_FRAME_SIZE];
    build_frame(frame, data);

    if (!transact(port, frame, resp)) {
        return false;
    }
    odom->mileage_laps = (int32_t)(((uint32_t)resp[2] << 24) | ((uint32_t)resp[3] << 16) |
                                   ((uint32_t)resp[4] << 8) | resp[5]);
    odom->position = (uint16_t)(((uint16_t)resp[6] << 8) | resp[7]);
    odom->error_code = resp[8];
    return true;
}

// -- Protocol 3: set mode (0xA0) --------------------------------------------

bool ddsm210_set_mode(ddsm_port_id_t port, uint8_t motor_id, uint8_t mode) {
    uint8_t data[9] = {motor_id, 0xA0, mode, 0, 0, 0, 0, 0, 0};
    uint8_t frame[DDSM_FRAME_SIZE];
    uint8_t resp[DDSM_FRAME_SIZE];
    build_frame(frame, data);

    if (transact(port, frame, resp)) {
        return true;
    }
    // Mode switch may not return feedback (per Waveshare docs); send once
    // more blind so the command definitely went out.
    send_no_response(port, frame);
    return false;
}

// -- Protocol 5: query mode (0x75) ------------------------------------------

int ddsm210_get_mode(ddsm_port_id_t port, uint8_t motor_id) {
    uint8_t data[9] = {motor_id, 0x75, 0, 0, 0, 0, 0, 0, 0};
    uint8_t frame[DDSM_FRAME_SIZE];
    uint8_t resp[DDSM_FRAME_SIZE];
    build_frame(frame, data);

    if (!transact(port, frame, resp)) {
        return -1;
    }
    return resp[2];
}
