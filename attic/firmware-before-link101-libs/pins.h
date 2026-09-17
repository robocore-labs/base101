#ifndef AXON_PINOUT_H
#define AXON_PINOUT_H

// ============================================================
// RoboCore Axon — rev2 board, RP2350A (30 GPIO, GP0..GP29)
//
// This revision drops the rev1 RS485 / CAN / second-UART / NeoPixel
// hardware. The firmware drives:
//   - 4x DDSM210 wheel motors, one PIO UART each   (GP19..GP26)
//   - 1x unified Feetech/Dynamixel half-duplex servo bus (GP7/8 + TXEN)
//   - 1x RPLidar on the uart1 hardware peripheral   (GP4/5)
//   - BNO055 IMU on i2c1                             (GP14/15)
//   - config/sniff button                           (GP17)
//   - 6x WS2812 NeoPixel mode indicator             (GP18)
//
// All four DDSM UART pairs were validated by PIO-UART loopback on this
// board (GP19->20, 21->22, 23->24, 25->26 all 6/6 OK).
// ============================================================

// --- Lidar (RPLidar C1) — hardware uart1 ---
// GP4 = UART1 TX, GP5 = UART1 RX (RP2350 function select F2).
#define PIN_LIDAR_TX         4
#define PIN_LIDAR_RX         5

// --- Servo bus (unified Feetech STS/SCS + Dynamixel, half-duplex, PIO) ---
// One PIO UART (pio0 SM0/SM1) with an external direction-enable line.
#define PIN_MOTOR_TX         7
#define PIN_MOTOR_RX         8
#define PIN_MOTOR_TXEN      16   // TX enable / bus direction control

// --- I2C (BNO055 IMU, Qwiic) — i2c1 ---
#define PIN_IMU_SDA         14
#define PIN_IMU_SCL         15

// --- Config / bus-sniff button (held LOW at boot -> config mode) ---
#define PIN_CONFIG_BUTTON   17

// --- WS2812 NeoPixel strip (mode indicator: red = config, blue = normal) ---
// Single data line, carried by the spare PIO0 SM2 (see src/led.c). Same wiring
// as rev1. Adjust NEOPIXEL_COUNT to the strip actually populated on the board.
#define PIN_NEOPIXELS       18
#define NEOPIXEL_COUNT      6

// --- DDSM210 wheel motors — four independent PIO UARTs ---
// Lower pin = MCU TX (-> motor RX), higher pin = MCU RX (<- motor TX).
// If a motor stays silent, swap TX/RX for that pair (board connector wiring).
//   FR -> pio1 SM0/SM1   FL -> pio1 SM2/SM3
//   BR -> pio2 SM0/SM1   BL -> pio2 SM2/SM3
#define PIN_DDSM_FR_TX      19
#define PIN_DDSM_FR_RX      20
#define PIN_DDSM_FL_TX      21
#define PIN_DDSM_FL_RX      22
#define PIN_DDSM_BR_TX      23
#define PIN_DDSM_BR_RX      24
#define PIN_DDSM_BL_TX      25
#define PIN_DDSM_BL_RX      26

// ============================================================
// PIO state-machine budget — RP2350 has 3 PIO blocks x 4 SM = 12 SMs.
//
//   PIO0:  servo half-duplex   SM0 TX (GP7)   SM1 RX (GP8)
//   PIO1:  DDSM FR             SM0 TX (GP19)  SM1 RX (GP20)
//          DDSM FL             SM2 TX (GP21)  SM3 RX (GP22)
//   PIO2:  DDSM BR             SM0 TX (GP23)  SM1 RX (GP24)
//          DDSM BL             SM2 TX (GP25)  SM3 RX (GP26)
//
//   Total: 10 of 12 SMs used (no gpio_base needed — every pin is GP0..29).
//   Lidar (uart1) and IMU (i2c1) use hardware peripherals, not PIO.
// ============================================================

#endif // AXON_PINOUT_H
