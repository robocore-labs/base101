# Axon ROS Node Firmware

Firmware for the **RoboCore Axon 2 board (RP2354B)** that turns it into a **native ROS 2 node**: the motor control for the [LLMy robot](https://github.com/cristidragomir97/LLMy) runs directly in firmware and is exposed to the ROS graph over **zenoh** ([Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software) + [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico), compatible with [rmw_zenoh](https://github.com/ros2/rmw_zenoh)).

This replaces the previous "multiprotocol USB bridge" firmware. The host no longer talks raw serial protocols to motors through forwarded CDC ports — it publishes and subscribes to ROS topics. The remaining passthrough is the lidar UART.

```
            ┌──────────────────────────── Axon 2 (RP2354B) ────────────────────────┐
            │                                                                       │
 USB CDC #0 │  zenoh serial transport ── Pico-ROS node "axon"                       │
◄──────────►│     sub /motor_manager/base_cmd ───► DDSM210 driver ──► 4× wheel UARTs │──► 4× DDSM210
            │     sub /motor_manager/arm_cmd  ───► ST3215 driver ──► merged servo bus│──► 6× ST3215
            │     pub /motor_manager/joint_states ◄── telemetry loop                 │
            │     pub /motor_telemetry/<joint>/* ◄── telemetry loop                  │
            │     pub /imu/data, /imu/mag, /imu/temperature ◄── BNO055 (i2c1)        │
 USB CDC #1 │                                                                       │
◄──────────►│  raw passthrough ◄──────────────────────────────────────► lidar uart1 │──► RPLidar C1
 USB CDC #2 │                                                                       │
◄──────────►│  debug log ◄── init info / discovered IDs / status                    │
            └───────────────────────────────────────────────────────────────────────┘
```

It is the firmware equivalent of the LLMy host packages [`ddsm210_manager`](https://github.com/cristidragomir97/LLMy/tree/main/ros/src/ddsm210_manager) and [`st3215_manager`](https://github.com/cristidragomir97/LLMy/tree/main/ros/src/st3215_manager) — same topics, same message types, same joint/index mapping.

---

## ROS interface

| Direction | Topic | Type | Function |
|---|---|---|---|
| sub | `/motor_manager/base_cmd` | `std_msgs/Float64MultiArray` | 4 wheel velocities, rad/s |
| sub | `/motor_manager/arm_cmd` | `std_msgs/Float64MultiArray` | arm joint angles, rad |
| sub | `/motor_manager/camera_cmd` | `std_msgs/Float64MultiArray` | camera tilt, rad (disabled by default) |
| pub | `/motor_manager/joint_states` | `sensor_msgs/JointState` | merged wheel + arm state, 50 Hz |
| pub | `/motor_telemetry/<joint>/current` | `std_msgs/Float32` | servo current, mA |
| pub | `/motor_telemetry/<joint>/voltage` | `std_msgs/Float32` | servo voltage, V |
| pub | `/motor_telemetry/<joint>/load` | `std_msgs/Float32` | servo load, % |
| pub | `/motor_telemetry/<joint>/temperature` | `std_msgs/Int32` | servo temperature, °C |
| pub | `/imu/data` | `sensor_msgs/Imu` | BNO055 fusion: orientation, angular velocity, linear accel, 50 Hz |
| pub | `/imu/mag` | `sensor_msgs/MagneticField` | BNO055 magnetometer, tesla |
| pub | `/imu/temperature` | `sensor_msgs/Temperature` | BNO055 chip temperature, °C |

Behaviour ported from the original packages:

- **Wheels (DDSM210, velocity loop).** Four independent wheels. `base_cmd[command_index] × direction` rad/s → RPM (`speed_scale` applied) → 0.1-RPM units, clamped to `max_rpm` (default 100, hardware max 210). The command array order is `[0] front_left, [1] front_right, [2] back_left, [3] back_right`; left side runs `direction = -1`, right side `+1`. Commands are deduplicated (only changes hit the bus). Joint-state position is the absolute multi-turn angle reconstructed from mileage laps + the 16-bit encoder.
- **Arm (ST3215, position mode).** `arm_cmd[command_index] × direction` rad → ticks (`2048 + angle/2π × 4096`, clamped to 0..4095), sent as `MoveTo` with the group's `position_speed`/`position_accel` scaled by `speed_scale`. On boot each servo is set to position mode, torque-enabled, and gently holds its current position (like the original manager).
- **Velocity-mode ST groups** (none enabled by default) implement the original behaviour: zero speed → `Rotate(0)` + torque off; non-zero → torque on + `Rotate`.
- **JointState layout.** One merged message with `AXON_TOTAL_JOINTS` (10) fixed slots; each motor writes its `state_index` (wheels 0–3, arm joints 4–9), unresponsive motors leave their slot empty — same `state_indices` convention as the YAML configs. (Camera tilt would be slot 10; enabling it requires bumping `AXON_TOTAL_JOINTS` to 11.)
- **IMU (BNO055, NDOF fusion).** One I2C sample per 50 Hz tick published as `Imu` + `MagneticField` + `Temperature`. `linear_acceleration` includes gravity (per `sensor_msgs/Imu` convention); covariances are fixed nominal diagonals in `axon_config.h`. If the sensor doesn't answer at boot, the IMU topics are simply not declared and the rest of the node is unaffected.
- **Per-motor telemetry** reads one servo per tick round-robin so bus time per control cycle stays bounded. Joint names that start with a digit get the `joint_` topic prefix (`/motor_telemetry/joint_1/current`), as in the original.

All motor mapping lives in **`src/ros/axon_config.h`** — motor IDs, joint names, directions, command/state indices, speed scales, accelerations, rates, topics, IMU config, node name and domain ID. It mirrors `ddsm210.yaml` + `llmy.yaml`. Edit and reflash to change it.

Known differences from the host packages:

- `JointState.header.stamp` (and the IMU stamps) are **time since boot** (the board has no RTC / synchronized clock). Consumers that need wall-clock stamps should re-stamp on the host.
- The DDSM position-loop mode of `ddsm210_manager` is not wired up (LLMy uses velocity mode); the driver itself supports it (`ddsm210_set_position`).
- Startup test sequences (`test_on_startup`) and the configurable brake methods are not ported; velocity-mode stop uses the default `torque_disable` behaviour.

---

## Hardware

The board has **two physical servo connectors** (Feetech/STS + a "Dynamixel" connector). Both run the Feetech STS protocol and are presented to the firmware as **one logical servo bus**: each command is broadcast to both channels and replies are merged (servo IDs must be unique across the two connectors). Each DDSM210 gets a dedicated UART because the motor cannot share a TX line without external gating.

| Bus | Pins (`pins.h`) | Notes |
|---|---|---|
| Servo bus A (Feetech/STS) | TX 2, RX 3, TXEN 15 | Feetech half-duplex @ 1 Mbaud, PIO0 SM0/1 |
| Servo bus B (Dynamixel conn.) | TX 12, RX 13, TXEN 14 | same protocol, PIO0 SM2/3 — merged with bus A |
| DDSM210 front-right / front-left | 32/33, 34/35 | 115200 8N1, PIO1 |
| DDSM210 back-right / back-left | 36/37, 38/39 | 115200 8N1, PIO2 |
| Lidar UART | TX 4, RX 5 | hardware `uart1`, default 460800 (RPLidar C1) |
| BNO055 IMU | SDA 26, SCL 27 | hardware `i2c1`, addr 0x28 |
| Debug / stdio UART | TX 0, RX 1 | hardware `uart0` |
| Activity LEDs | 30, 31, 21–24 | classic GPIO (PWM-dimmed), single core — see caveat |

The PIO budget lands exactly at 12/12: PIO0 = the two servo buses (4 SMs), PIO1 + PIO2 = the four wheel motors (8 SMs). The lidar uses the `uart1` hardware peripheral and the LEDs are plain GPIO, so neither consumes a state machine.

> **Pin caveats (confirm against the board harness):**
> - The board routes some activity LEDs to GP34–37, but those pins are taken by the DDSM motors. `LED_FEETECH` (GP30) and `LED_I2C` (GP31) are correct; `LED_UART0`/`UART1`/`CAN`/`RS485` are **placeholders on GP21–24**. Wrong LED pins are cosmetic, not fatal.
> - Lidar wiring assumes firmware TX = GP4 (uart1 TX) / RX = GP5. Swap the `PIN_LIDAR_*` defines if the connector is reversed.

USB layout (VID:PID `1209:AC01`):

| Interface | Name | Function |
|---|---|---|
| CDC #0 | `RoboCore Axon Zenoh` | zenoh serial transport |
| CDC #1 | `RoboCore Axon Lidar` | lidar UART passthrough |
| CDC #2 | `RoboCore Axon Debug` | debug log (init info, discovered IDs, status) |

The board also carries a CAN controller (MCP2518FD on SPI) and an RS485 transceiver that this firmware does not currently drive.

---

## Building

Requires the [pico-sdk](https://github.com/raspberrypi/pico-sdk) (2.x) and an `arm-none-eabi` toolchain. The build selects the RP2350**B** part (`PICO_RP2350A=0` in `CMakeLists.txt`) so GP32–39 are valid.

```bash
export PICO_SDK_PATH=~/pico/pico-sdk
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

Flash by holding BOOT, plugging in, and copying `build/axon_firmware.uf2` to the `RPI-RP2` drive (or `picotool load -f build/axon_firmware.uf2`).

Debug/init logs go to **USB CDC #2** (and mirror to the stdio UART on GP0/1); per-message traffic is *not* logged there.

---

## Host setup

### 1. udev rules

```bash
./install.sh
```

gives you stable symlinks:

- `/dev/axon-zenoh` — zenoh serial transport
- `/dev/axon-lidar` — lidar passthrough
- `/dev/axon-debug` — debug log (open with any serial terminal, e.g. `screen /dev/axon-debug 115200`)

### 2. zenoh router with serial transport

The firmware is a zenoh **client** that connects through its serial link; the host must run a zenoh router listening on that serial port. Serial transport is **not enabled in stock zenohd builds** — you need a zenohd compiled with `--features transport_serial`.

**Easiest: the Docker image in [`docker/`](docker/)** rebuilds zenohd with that feature on top of `eclipse/zenoh:latest`:

```bash
cd docker
docker compose up --build      # maps /dev/axon-zenoh, listens serial + tcp/7447
```

**From source** instead:

```bash
git clone https://github.com/eclipse-zenoh/zenoh && cd zenoh
cargo build --release -p zenohd --features transport_serial
# the baudrate token is required by the locator syntax but meaningless on USB CDC
./target/release/zenohd -l 'serial//dev/axon-zenoh#baudrate=921600'
```

Keep the router protocol-compatible with the firmware's vendored zenoh-pico (currently **1.9.0**, see `lib/zenoh-pico/include/zenoh-pico.h`).

### 3. ROS 2 with rmw_zenoh

If the serial `zenohd` is your only router, point rmw_zenoh's sessions at it; otherwise federate it with your existing `rmw_zenohd` router:

```bash
# option A: single router — also listen on the default tcp port rmw_zenoh expects
zenohd -l 'serial//dev/axon-zenoh#baudrate=921600' -l 'tcp/[::]:7447'

# option B: keep rmw_zenohd and bridge the serial router into it
zenohd -l 'serial//dev/axon-zenoh#baudrate=921600' -e 'tcp/localhost:7447'
```

Then, in any terminal with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`:

```bash
ros2 topic list
ros2 topic echo /motor_manager/joint_states
ros2 topic echo /imu/data
# spin all four wheels at 1 rad/s: [front_left, front_right, back_left, back_right]
ros2 topic pub -r 20 /motor_manager/base_cmd std_msgs/msg/Float64MultiArray '{data: [1.0, 1.0, 1.0, 1.0]}'
# move the arm to home
ros2 topic pub --once /motor_manager/arm_cmd std_msgs/msg/Float64MultiArray '{data: [0, 0, 0, 0, 0, 0]}'
```

### 4. Lidar

Point the existing driver at the passthrough port, exactly as before:

```bash
ros2 launch rplidar_ros rplidar_c1_launch.py serial_port:=/dev/axon-lidar
```

---

## Configuration & test mode

The DDSM wheels are fixed in firmware, but the **ST3215 servos are runtime
configurable** (how many, their bus IDs and joint names, and whether the servo
subsystem runs at all). The config lives in flash and is loaded at boot;
`servos_enabled` defaults to **off**, so out of the box the board is DDSM-only
and `joint_states` carries just the 4 wheels.

**Entering config mode:** hold the **sniff/config button** (GP32) while powering
on. The board comes up with USB + motor buses but does **not** start
zenoh/lidar — it serves an interactive JSON console on the debug port (CDC #2,
`/dev/axon-debug`). Without the button it boots normally and the debug port is
status-only.

The 6 WS2812 NeoPixels (GP18) are a mode-at-a-glance indicator: **all breathing
red** = config mode, **all breathing blue** = normal ROS operation. A frozen
strip means frozen firmware.

**Console** (one JSON object per line on `/dev/axon-debug`):

| Command | Effect |
|---|---|
| `{"cmd":"get"}` | print current config |
| `{"cmd":"set","servos_enabled":true,"servos":[{"id":1,"joint":"shoulder","enable":true}, …]}` | update config in RAM |
| `{"cmd":"save"}` / `{"cmd":"reboot"}` | persist to flash / restart (apply) |
| `{"cmd":"defaults"}` | reset config to compiled defaults |
| `{"cmd":"motors"}` | list wheels + servos with online status |
| `{"cmd":"wheel","index":0,"rpm":30}` / `{"cmd":"stop"}` | spin a wheel / stop all wheels |
| `{"cmd":"servo","id":1,"pos":2048}` or `"delta":256` | move a servo (absolute ticks / relative) |
| `{"cmd":"setid","from":1,"to":5}` | change a servo's bus ID (one servo on the bus) |

Config edits apply on the next boot (motor tables / publishers are built at
startup), so the flow is **set → save → reboot**.

**Web UI:** open `web/axon-config.html` in Chrome/Edge, click *Connect* and pick
the debug serial port. It edits the servo list, lists and test-drives motors
(spin wheels / step servos), and changes servo IDs — all over Web Serial, no
install. It's just a static file; open it directly or serve the `web/` folder.

---

## Firmware architecture

### Layout

```
main.c                      main loop: zenoh rx → I/O poll → telemetry
src/ros/axon_config.h       ALL robot configuration (motors, IMU, topics, rates)
src/ros/axon_node.c/.h      Pico-ROS node: subscribers, publishers, conversions
src/ros/axon_types.h        picoserdes message definitions (names + RIHS01 hashes)
src/motors/ddsm210.c/.h     DDSM210 protocol (CRC8-MAXIM 10-byte frames)
src/motors/st3215.c/.h      Feetech STS protocol (ping/mode/torque/move/telemetry)
src/sensors/bno055.c/.h     BNO055 IMU (I2C, NDOF fusion)
src/bus/half_duplex.c/.h    merged two-channel Feetech servo bus + transact helper
src/bus/ddsm_port.c/.h      dedicated PIO UART per DDSM motor (PIO1/PIO2)
src/bus/lidar_uart.c/.h     lidar on hardware uart1 (forwarded to CDC #1)
src/led.c/.h                WS2812 NeoPixel mode indicator (red=config, blue=normal), PIO0 SM2
src/dbg.c/.h                debug logging to USB CDC #2
src/zenoh_port/             bare-metal zenoh-pico platform port (see below)
lib/zenoh-pico/             vendored zenoh-pico (pico-ros pinned revision)
lib/picoros/                vendored Pico-ROS (picoros + picoserdes)
lib/microcdr/               vendored eProsima Micro-CDR
docker/                     serial-enabled zenohd image (host router)
```

### The zenoh-pico port (`src/zenoh_port/`)

zenoh-pico's stock Raspberry Pi Pico platform requires FreeRTOS and owns the USB descriptors for its serial-over-USB link — both incompatible with this bare-metal composite-USB firmware. Instead the build uses a custom platform profile (`lib/zenoh-pico/cmake/platforms/axon.cmake`, the one file added to the vendored tree):

- **Single-threaded** (`Z_FEATURE_MULTI_THREAD=0`): no tasks, no mutexes. The main loop calls `picoros_single_threaded_loop()` (zenoh rx + periodic keepalive).
- **`system_axon.c`**: malloc/random/clock/sleep on top of the pico-sdk. The zenoh configuration itself is in `zenoh_generic_config.h` (with a generic platform, zenoh-pico's CMake config values do not apply — that header is the single source of truth).
- **`serial_axon.c`**: implements zenoh's serial lower layer (`_z_serial_open_from_dev` for device `"cdc"`, read/write) on TinyUSB CDC #0. The zenoh serial protocol above it does COBS framing + CRC32; the same framing is spoken by `zenohd`'s serial transport.
- **Cooperative blocking**: zenoh's frame reader pulls one byte at a time. The port applies a short timeout at frame boundaries (so an idle link returns "no data" and the main loop keeps spinning at ~`AXON_ZENOH_POLL_TIMEOUT_MS`) and a long timeout mid-frame (so frames in flight aren't corrupted). While waiting, it runs a registered idle poll (`io_poll()` in main.c: TinyUSB, lidar bridge, bus RX rings, LED task) — which must never call back into zenoh.
- The session handshake gets a temporarily long poll timeout (`axon_zenoh_set_poll_timeout_ms`), and `picoros_interface_init` is retried until `zenohd` shows up, with motors already initialized and the lidar bridge live in the meantime.

Vendored-code changes (kept deliberately minimal):

1. `lib/zenoh-pico/cmake/platforms/axon.cmake` — added platform profile.
2. `lib/picoros/picoros.c` — `[axon patch]` guards: `zp_start_read_task` / `zp_start_lease_task` / `zp_*_task_is_running` only exist in multi-threaded zenoh-pico builds.

### Main loop timing

One iteration = one `zp_read` (≤ 2 ms idle poll timeout, longer only while frames are arriving) + I/O poll + due telemetry. Motor transactions have hard deadlines (DDSM 4 ms, ST3215 2 ms per attempt) and pump USB + RX rings while waiting, so the lidar stream and zenoh CDC FIFOs survive worst-case bus stalls. At 50 Hz the joint-state pass costs roughly: 4 DDSM odometry reads + 6 ST3215 state reads; the IMU pass adds one I2C sample.

---

## Tuning / extending

- **Change motor / IMU mapping, rates, topics**: `src/ros/axon_config.h`.
- **Add an ST3215 group**: add a motor table + group entry in `axon_config.h`, and a trampoline in `axon_node.c` (`group_trampolines`) if you exceed two groups.
- **Add a message type**: add its definition (name + RIHS01 hash) to `src/ros/axon_types.h`; type entries can be generated from `.msg` files with [pico-ros type-gen](https://github.com/Pico-ROS/Pico-ROS-software/tree/main/tools/type-gen).
- **ROS_DOMAIN_ID**: `AXON_ROS_DOMAIN_ID` in `axon_config.h` (must match the host).
- **Pin assignment**: `pins.h` (see the pin caveats under Hardware).
