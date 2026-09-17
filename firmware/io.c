#include "io.h"

#include "pico/stdlib.h"
#include "tusb.h"

#include "lidar.h"
#include "status.h"
#include "usb_descriptors.h"

void io_begin(void) {
    tusb_init();

    // Wait for enumeration, then for a reader on the debug port -- but not
    // for long, and never forever: the robot has to boot with no host
    // attached at all.
    uint32_t started = time_us_32();
    while (!tud_ready() && (time_us_32() - started) < 5000000) {
        tud_task();
        sleep_ms(10);
    }
    for (int i = 0; i < 80 && !tud_cdc_n_connected(CDC_IDX_DEBUG); i++) {
        tud_task();
        sleep_ms(10);
    }
}

void io_poll(void) {
    tud_task();
    lidar_update();
    status_update();   // so the LED keeps breathing even while zenoh blocks
}

// The host changed the baud rate on a CDC port. Only the lidar port has a
// real UART behind it, where the setting means something.
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
    if (itf == CDC_IDX_LIDAR) {
        lidar_set_baudrate(coding->bit_rate);
    }
}
