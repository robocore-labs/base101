#include "bno055.h"

#include "pico/stdlib.h"

// --- Register map (page 0) -------------------------------------------------
#define REG_CHIP_ID      0x00
#define REG_PAGE_ID      0x07
#define REG_ACC_DATA     0x08   // ACC x/y/z, MAG x/y/z, GYR x/y/z are
#define REG_MAG_DATA     0x0E   // contiguous int16 LSB-first from 0x08..0x19
#define REG_GYR_DATA     0x14
#define REG_QUA_DATA     0x20   // W, X, Y, Z int16 LSB-first
#define REG_TEMP         0x34
#define REG_UNIT_SEL     0x3B
#define REG_OPR_MODE     0x3D
#define REG_PWR_MODE     0x3E
#define REG_SYS_TRIGGER  0x3F

#define CHIP_ID_VALUE    0xA0

#define OPR_MODE_CONFIG  0x00
#define OPR_MODE_NDOF    0x0C
#define PWR_MODE_NORMAL  0x00

// LSB-per-unit scale factors in the default unit selection.
#define LSB_PER_MS2      100.0f   // accel: 100 LSB = 1 m/s^2
#define LSB_PER_UT       16.0f    // mag:   16 LSB  = 1 uT
#define LSB_PER_DPS      16.0f    // gyro:  16 LSB  = 1 deg/s
#define LSB_PER_QUAT     16384.0f // quaternion: 2^14 LSB = 1.0
#define UT_TO_TESLA      1.0e-6f
#define DEG_TO_RAD       0.01745329251994329577f

static i2c_inst_t *bno_i2c = NULL;
static uint8_t bno_addr = 0;

static bool reg_write(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(bno_i2c, bno_addr, buf, 2, false) == 2;
}

static bool reg_read(uint8_t reg, uint8_t *dst, size_t n) {
    if (i2c_write_blocking(bno_i2c, bno_addr, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(bno_i2c, bno_addr, dst, n, false) == (int)n;
}

static inline int16_t le16(const uint8_t *p) {
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

bool bno055_init(i2c_inst_t *i2c, uint8_t addr) {
    bno_i2c = i2c;
    bno_addr = addr;

    // The BNO055 can take up to ~650 ms after power-on before it answers.
    uint8_t id = 0;
    bool found = false;
    for (int i = 0; i < 15; i++) {
        if (reg_read(REG_CHIP_ID, &id, 1) && id == CHIP_ID_VALUE) {
            found = true;
            break;
        }
        sleep_ms(50);
    }
    if (!found) {
        return false;
    }

    // Settings can only be changed in CONFIG mode.
    reg_write(REG_OPR_MODE, OPR_MODE_CONFIG);
    sleep_ms(25);

    reg_write(REG_PAGE_ID, 0x00);
    reg_write(REG_PWR_MODE, PWR_MODE_NORMAL);
    sleep_ms(10);
    reg_write(REG_SYS_TRIGGER, 0x00);   // use the internal oscillator
    // UNIT_SEL = 0: accel m/s^2, gyro deg/s, euler deg, temp C, Windows orient.
    reg_write(REG_UNIT_SEL, 0x00);
    sleep_ms(10);

    // Enter fusion mode. CONFIG -> operating mode switch needs >= 7 ms.
    reg_write(REG_OPR_MODE, OPR_MODE_NDOF);
    sleep_ms(20);
    return true;
}

bool bno055_read(bno055_sample_t *out) {
    if (!bno_i2c) {
        return false;
    }

    // ACC(6) + MAG(6) + GYR(6) are contiguous from 0x08.
    uint8_t amg[18];
    if (!reg_read(REG_ACC_DATA, amg, sizeof(amg))) {
        return false;
    }
    for (int i = 0; i < 3; i++) {
        out->accel[i] = le16(&amg[0 + 2 * i]) / LSB_PER_MS2;
        out->mag[i]   = (le16(&amg[6 + 2 * i]) / LSB_PER_UT) * UT_TO_TESLA;
        out->gyro[i]  = (le16(&amg[12 + 2 * i]) / LSB_PER_DPS) * DEG_TO_RAD;
    }

    uint8_t q[8];
    if (!reg_read(REG_QUA_DATA, q, sizeof(q))) {
        return false;
    }
    for (int i = 0; i < 4; i++) {
        out->quat[i] = le16(&q[2 * i]) / LSB_PER_QUAT;  // order W, X, Y, Z
    }

    uint8_t t = 0;
    if (!reg_read(REG_TEMP, &t, 1)) {
        return false;
    }
    out->temp_c = (int8_t)t;
    return true;
}
