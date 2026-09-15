/*
 * Axon ROS Node Firmware
 * ======================
 *
 * The board (Axon 2 rev2 / RP2350A) is a native ROS 2 node: motor control for
 * the LLMy robot runs in firmware and is exposed over zenoh (Pico-ROS +
 * zenoh-pico, rmw_zenoh compatible) carried on USB CDC #0.
 *
 *   USB Interface     Function
 *   ------------------------------------------------------
 *   CDC #0            zenoh serial transport (Pico-ROS)
 *   CDC #1            Lidar UART passthrough
 *   CDC #2            Debug log (init info, IDs, status)
 *
 *   Bus               Hardware
 *   ------------------------------------------------------
 *   Servo bus         Feetech half-duplex, single channel (PIO0, GP7/8 + TXEN GP16)
 *   DDSM210 x4        one PIO UART per wheel motor (PIO1/PIO2, GP19..GP26)
 *   Lidar UART        hardware uart1 (GP4/5)
 *   IMU               BNO055 on i2c1 (GP14/15)
 *
 * ROS topics (via zenohd + rmw_zenoh on the host):
 *   sub  /motor_manager/base_cmd      std_msgs/Float64MultiArray
 *   sub  /motor_manager/arm_cmd       std_msgs/Float64MultiArray
 *   pub  /motor_manager/joint_states  sensor_msgs/JointState
 *   pub  /motor_telemetry/<joint>/*   Float32 / Int32
 *   pub  /imu/data, /imu/mag, /imu/temperature
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "tusb.h"

#include "picoros.h"

#include "pins.h"
#include "usb_descriptors.h"
#include "dbg.h"
#include "led.h"
#include "bus/half_duplex.h"
#include "bus/lidar_uart.h"
#include "bus/ddsm_port.h"
#include "config/axon_cfg.h"
#include "config/cfg_console.h"
#include "ros/axon_config.h"
#include "ros/axon_node.h"
#include "zenoh_port/axon_zenoh.h"

//--------------------------------------------------------------------
// Lidar passthrough: CDC #1 <-> lidar UART
//--------------------------------------------------------------------
static void lidar_bridge(void) {
    uint8_t buf[64];

    // USB -> Lidar
    if (tud_cdc_n_available(CDC_IDX_LIDAR)) {
        uint32_t count = tud_cdc_n_read(CDC_IDX_LIDAR, buf, sizeof(buf));
        if (count > 0) lidar_uart_write(buf, count);
    }

    // Lidar -> USB
    while (lidar_uart_available() > 0 && tud_cdc_n_write_available(CDC_IDX_LIDAR) > 0) {
        uint32_t want = tud_cdc_n_write_available(CDC_IDX_LIDAR);
        if (want > sizeof(buf)) want = sizeof(buf);
        uint32_t count = lidar_uart_read(buf, want);
        if (count == 0) break;
        tud_cdc_n_write(CDC_IDX_LIDAR, buf, count);
    }
    tud_cdc_n_write_flush(CDC_IDX_LIDAR);
}

//--------------------------------------------------------------------
// Background I/O poll
//
// Runs from the main loop and from inside the zenoh port whenever it
// waits on serial data or sleeps. Never touches zenoh.
//--------------------------------------------------------------------
static void io_poll(void) {
    tud_task();
    half_duplex_task();
    ddsm_port_task();
    lidar_uart_task();
    lidar_bridge();
    led_task();  // keeps the mode indicator breathing even while zenoh blocks
}

//--------------------------------------------------------------------
// Config mode
//
// Entered when the sniff/config button (PIN_CONFIG_BUTTON) is held HIGH at
// boot (active-high, idles low). Brings up USB and serves the JSON console on CDC #2;
// motors, lidar and zenoh are intentionally NOT started, so the robot stays
// still while it is reconfigured. Exits only by reboot (e.g. the console's
// {"cmd":"reboot"}). Never returns.
//--------------------------------------------------------------------
// noinline + a volatile capture so the optimizer treats the button state as a
// genuine runtime value. (Without this, -O3 inlined the noreturn config loop
// and collapsed the branch, dropping the entire normal-mode tail of main.)
//
// The config button on rev2 is ACTIVE-HIGH: it idles LOW (external pull-down)
// and is pressed by pulling GP17 up to 3V3. So config mode is entered only when
// the line is held HIGH for essentially the whole sampling window. We enable the
// internal pull-DOWN to match (reinforces the idle-low resting state), so a
// disconnected/floating line stays in NORMAL mode. Debounced over 32 samples so
// a momentary glitch can't trip it. The tally is stashed so main() can narrate
// it on the debug port once USB is up.
static uint8_t g_btn_high_samples;
static uint8_t g_btn_total_samples;
static bool __attribute__((noinline)) config_button_pressed(void) {
    gpio_init(PIN_CONFIG_BUTTON);
    gpio_set_dir(PIN_CONFIG_BUTTON, GPIO_IN);
    gpio_pull_down(PIN_CONFIG_BUTTON);  // idle LOW; button pulls it HIGH (3V3)
    sleep_ms(5);  // let the pull-down settle

    g_btn_total_samples = 32;
    g_btn_high_samples = 0;
    for (uint8_t i = 0; i < g_btn_total_samples; i++) {
        if (gpio_get(PIN_CONFIG_BUTTON) == 1) g_btn_high_samples++;
        sleep_ms(1);
    }
    gpio_deinit(PIN_CONFIG_BUTTON);  // release the pull-down after the read
    // Require an almost-fully-held HIGH window (>=30/32) to enter config mode.
    volatile bool pressed = (g_btn_high_samples >= 30);
    return pressed;
}

// USB is already up (main() brings it up before deciding the mode), so this
// just brings up the motor buses and serves the console. Never returns.
static void __attribute__((noinline)) run_config_mode(void) {
    dbg_printf("[cfg ] CONFIG MODE — JSON console on CDC #2; zenoh/lidar/IMU OFF\n");
    dbg_printf("[cfg ] cmds: get|set|save|defaults|reboot ; motors|wheel|servo|setid|stop\n");

    // Bring up the motor buses so the bench test commands ("motors"/"wheel"/
    // "servo"/"stop") can drive motors. zenoh/lidar/IMU stay off.
    half_duplex_init(AXON_ST_BAUD);
    ddsm_port_init(AXON_DDSM_BAUD);
    led_init(LED_MODE_CONFIG);  // all pixels breathe red while in config mode

    axon_cfg_load();
    axon_node_motors_init();  // detect motors + set modes (servos per config)
    dbg_printf("[cfg ] servos %s, %u configured. Ready.\n",
               g_cfg.servos_enabled ? "enabled" : "disabled", g_cfg.servo_count);

    while (true) {
        tud_task();
        cfg_console_task();
        led_task();
    }
}

// Host changed the baudrate of a CDC port: forward to the lidar UART.
void tud_cdc_line_coding_cb(uint8_t itf, cdc_line_coding_t const *coding) {
    if (itf == CDC_IDX_LIDAR) {
        lidar_uart_set_baudrate(coding->bit_rate);
    }
}

//--------------------------------------------------------------------
// Main
//--------------------------------------------------------------------
int main(void) {
    stdio_init_all();

    // Decide the mode FIRST (debounced GP17 read), but don't act until USB is
    // up so a host watching the debug CDC sees the entire boot narration.
    bool want_config = config_button_pressed();

    // Bring USB up before anything else, in BOTH modes, so the debug port is
    // available for narration. Then give a host a brief window to open it.
    tusb_init();
    uint32_t usb_start = time_us_32();
    while (!tud_ready() && (time_us_32() - usb_start) < 5000000) {
        tud_task();
        sleep_ms(10);
    }
    for (int i = 0; i < 80 && !tud_cdc_n_connected(CDC_IDX_DEBUG); i++) {
        tud_task();
        sleep_ms(10);
    }

    dbg_printf("\n\n=== Axon ROS Node boot (rev2 / RP2350A) ===\n");
    dbg_printf("[boot] USB %s | CDC0=zenoh CDC1=lidar CDC2=debug\n",
               tud_ready() ? "enumerated" : "enumeration timeout (continuing)");
    dbg_printf("[boot] config button GP%d: %u/%u samples HIGH -> %s MODE\n",
               PIN_CONFIG_BUTTON, g_btn_high_samples, g_btn_total_samples,
               want_config ? "CONFIG" : "NORMAL");

    // Config/sniff button held HIGH at boot -> interactive config mode (never
    // returns). Otherwise fall through to normal operation.
    if (want_config) {
        run_config_mode();
    }

    // ---- NORMAL MODE -------------------------------------------------------
    // Buses. half_duplex_init() loads the PIO0 uart programs for the servo
    // bus; ddsm_port_init() loads its own on PIO1/PIO2; the lidar is on the
    // uart1 hardware peripheral.
    dbg_printf("[init] servo bus (Feetech/STS) @%u baud on GP%d/%d TXEN%d\n",
               (unsigned)AXON_ST_BAUD, PIN_MOTOR_TX, PIN_MOTOR_RX, PIN_MOTOR_TXEN);
    half_duplex_init(AXON_ST_BAUD);
    dbg_printf("[init] lidar uart1 @%u baud on GP%d/%d\n",
               (unsigned)AXON_LIDAR_DEFAULT_BAUD, PIN_LIDAR_TX, PIN_LIDAR_RX);
    lidar_uart_init(AXON_LIDAR_DEFAULT_BAUD);
    dbg_printf("[init] DDSM bus @%u baud on GP%d..GP%d (4 PIO UARTs)\n",
               (unsigned)AXON_DDSM_BAUD, PIN_DDSM_FR_TX, PIN_DDSM_BL_RX);
    ddsm_port_init(AXON_DDSM_BAUD);
    dbg_printf("[init] NeoPixel indicator @GP%d (%d px) -> breathing blue\n",
               PIN_NEOPIXELS, NEOPIXEL_COUNT);
    led_init(LED_MODE_NORMAL);  // all pixels breathe blue in normal ROS operation

    // Runtime config (servo list / enable) from flash, else compiled defaults.
    bool cfg_stored = axon_cfg_load();
    dbg_printf("[cfg ] %s; servos %s (%u configured)\n",
               cfg_stored ? "loaded from flash" : "compiled defaults",
               g_cfg.servos_enabled ? "enabled" : "disabled", g_cfg.servo_count);

    // Motors and IMU can be initialized regardless of the host being up. Each
    // probe logs online/offline per device (see axon_node).
    dbg_printf("[scan] probing motors (DDSM wheels + ST servos)...\n");
    axon_node_motors_init();
    dbg_printf("[scan] probing IMU (BNO055)...\n");
    axon_imu_init();

    // zenoh session over CDC #0. Generous poll timeout during the
    // handshake; the idle poll keeps USB + lidar + motor buses alive
    // while zenoh blocks.
    dbg_printf("[zenoh] locator '%s'; connecting to router...\n", AXON_ZENOH_LOCATOR);
    axon_zenoh_set_idle_poll(io_poll);
    axon_zenoh_set_poll_timeout_ms(1000);

    picoros_interface_t ifx = {
        .mode = AXON_ZENOH_MODE,
        .locator = AXON_ZENOH_LOCATOR,
    };

    uint32_t zattempt = 0;
    while (picoros_interface_init(&ifx) != PICOROS_OK) {
        if ((zattempt++ % 5) == 0) {
            dbg_printf("[zenoh] waiting for router (is zenohd serial endpoint up?) attempt %u\n",
                       (unsigned)zattempt);
        }
        for (int i = 0; i < 100; i++) {  // ~1 s, keeping I/O alive
            io_poll();
            sleep_ms(10);
        }
    }
    dbg_printf("[zenoh] session UP\n");

    axon_zenoh_set_poll_timeout_ms(AXON_ZENOH_POLL_TIMEOUT_MS);

    dbg_printf("[ros ] declaring node '%s' (domain %u)...\n",
               AXON_NODE_NAME, AXON_ROS_DOMAIN_ID);
    if (!axon_node_declare()) {
        dbg_printf("[ros ] FATAL: node declaration failed\n");
        while (true) {
            io_poll();
            sleep_ms(100);
        }
    }
    dbg_printf("[ros ] up; entering main loop. Silencing debug output now to "
               "free USB bandwidth for the zenoh transport.\n");
    dbg_enable_usb(false);  // no debug output past this point (steady state)

    // Main loop: zenoh rx (bounded by the poll timeout) + telemetry.
    while (true) {
        picoros_single_threaded_loop(&ifx);
        io_poll();
        axon_node_spin();
    }
}
