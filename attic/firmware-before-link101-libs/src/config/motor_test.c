#include "motor_test.h"

#include "../dbg.h"
#include "axon_cfg.h"
#include "../ros/axon_config.h"
#include "../motors/ddsm210.h"
#include "../motors/st3215.h"

// ST bus ID sweep range for discovery. Servos usually live at low IDs;
// 1..30 keeps the scan snappy. Widen if you address servos higher.
#define SCAN_SERVO_ID_MIN 1
#define SCAN_SERVO_ID_MAX 30

// DDSM bus ID sweep: each motor is alone on its bus and ships at the factory
// default ID 1; 1..10 mirrors the working reference terminal (do_scan).
#define SCAN_DDSM_ID_MAX 10

static uint16_t servo_speed(void) {
    return (uint16_t)((double)AXON_SERVO_POSITION_SPEED * AXON_SERVO_SPEED_SCALE);
}

// Physical DDSM connector label for a port — hardware identity, not a ROS
// joint name, so discovery stays independent of the configured motor map.
static const char *ddsm_port_label(ddsm_port_id_t p) {
    switch (p) {
        case DDSM_FR: return "FR";
        case DDSM_FL: return "FL";
        case DDSM_BR: return "BR";
        case DDSM_BL: return "BL";
        default:      return "?";
    }
}

// Find the motor on a bus the way the proven python terminal does: sweep
// get_mode() over candidate IDs and take the first that answers. Returns the
// ID (and its mode via out_mode) or -1. Does NOT use the 0xC8 ID-query, which
// not all motor firmware implements.
static int ddsm_scan_bus(ddsm_port_id_t port, int *out_mode) {
    for (int id = 1; id <= SCAN_DDSM_ID_MAX; id++) {
        int mode = ddsm210_get_mode(port, (uint8_t)id);
        if (mode >= 0) {
            if (out_mode) *out_mode = mode;
            return id;
        }
    }
    return -1;
}

// Pure bus discovery: probe every DDSM bus and sweep the ST bus, reporting
// whatever actually answers. No joint names or configured IDs are used — this
// reflects the hardware on the bus right now, not the ROS configuration.
void motor_test_list_json(void) {
    dbg_printf("{\"motors\":{\"wheels\":[");
    for (int i = 0; i < AXON_DDSM_COUNT; i++) {
        ddsm_port_id_t port = AXON_DDSM_MOTORS[i].port;
        int mode = -1;
        int found = ddsm_scan_bus(port, &mode);   // sweep IDs 1..10
        dbg_printf("%s{\"index\":%d,\"port\":\"%s\",\"online\":%s,\"id\":%d,\"mode\":%d}",
                   i ? "," : "", i, ddsm_port_label(port),
                   found >= 0 ? "true" : "false", found, mode);
    }
    dbg_printf("],\"servos\":[");
    bool first = true;
    for (int id = SCAN_SERVO_ID_MIN; id <= SCAN_SERVO_ID_MAX; id++) {
        if (st3215_ping((uint8_t)id)) {
            dbg_printf("%s{\"id\":%d}", first ? "" : ",", id);
            first = false;
        }
    }
    dbg_printf("],\"servo_scan\":[%d,%d]}}\n", SCAN_SERVO_ID_MIN, SCAN_SERVO_ID_MAX);
}

void motor_test_wheel(int index, double rpm) {
    if (index < 0 || index >= AXON_DDSM_COUNT) {
        dbg_printf("{\"ok\":false,\"msg\":\"bad wheel index\"}\n");
        return;
    }
    ddsm_port_id_t port = AXON_DDSM_MOTORS[index].port;
    // Drive whatever motor is on this bus, discovered live, rather than the
    // configured ID — so the bench tests the hardware as found.
    int id = ddsm_scan_bus(port, NULL);
    if (id < 0) {
        dbg_printf("{\"ok\":false,\"msg\":\"no motor on bus %d (%s)\"}\n",
                   index, ddsm_port_label(port));
        return;
    }

    int32_t rpm_x10 = (int32_t)(rpm * 10.0);
    int32_t cap = AXON_DDSM_MAX_RPM * 10;
    if (cap > DDSM210_MAX_SPEED_RAW) cap = DDSM210_MAX_SPEED_RAW;
    if (rpm_x10 > cap) rpm_x10 = cap;
    if (rpm_x10 < -cap) rpm_x10 = -cap;

    ddsm210_set_mode(port, (uint8_t)id, DDSM210_MODE_VELOCITY);
    ddsm210_set_velocity(port, (uint8_t)id, (int16_t)rpm_x10, AXON_DDSM_ACCEL_TIME, NULL);
    dbg_printf("{\"ok\":true,\"msg\":\"wheel %d (id %d) -> %d.%d rpm\"}\n",
               index, id, rpm_x10 / 10, (rpm_x10 < 0 ? -rpm_x10 : rpm_x10) % 10);
}

void motor_test_servo(int id, int pos) {
    if (pos < 0) pos = 0;
    if (pos > AXON_ST_TICKS_PER_REV - 1) pos = AXON_ST_TICKS_PER_REV - 1;

    st3215_set_mode((uint8_t)id, ST3215_MODE_POSITION);
    st3215_set_torque((uint8_t)id, true);
    st3215_move_to((uint8_t)id, (uint16_t)pos, servo_speed(), AXON_SERVO_DEFAULT_ACCEL);
    dbg_printf("{\"ok\":true,\"msg\":\"servo %d -> %d\"}\n", id, pos);
}

void motor_test_servo_delta(int id, int delta) {
    int32_t cur = st3215_read_position((uint8_t)id);
    if (cur < 0) {
        dbg_printf("{\"ok\":false,\"msg\":\"servo %d not responding\"}\n", id);
        return;
    }
    motor_test_servo(id, (int)cur + delta);
}

void motor_test_stop_all(void) {
    for (int i = 0; i < AXON_DDSM_COUNT; i++) {
        ddsm_port_id_t port = AXON_DDSM_MOTORS[i].port;
        int id = ddsm_scan_bus(port, NULL);
        if (id >= 0) {
            ddsm210_set_velocity(port, (uint8_t)id, 0, AXON_DDSM_ACCEL_TIME, NULL);
        }
    }
    dbg_printf("{\"ok\":true,\"msg\":\"all wheels stopped\"}\n");
}

void motor_test_set_servo_id(int old_id, int new_id) {
    if (new_id < 0 || new_id > 253) {
        dbg_printf("{\"ok\":false,\"msg\":\"id must be 0..253\"}\n");
        return;
    }
    bool ok = st3215_set_id((uint8_t)old_id, (uint8_t)new_id);
    dbg_printf("{\"ok\":%s,\"msg\":\"servo id %d -> %d%s\"}\n",
               ok ? "true" : "false", old_id, new_id,
               ok ? "" : " (no servo at old id, or change failed)");
}
