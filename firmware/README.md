# base101 firmware

Firmware for the **Link101 board (RP2350A)** that makes base101 a **ROS 2
node in its own right**. Motor control runs on the board; the host talks to
it over **zenoh** ([Pico-ROS](https://github.com/Pico-ROS/Pico-ROS-software)
+ [zenoh-pico](https://github.com/eclipse-zenoh/zenoh-pico), compatible with
[rmw_zenoh](https://github.com/ros2/rmw_zenoh)). There is no bridge process
and no serial protocol for the host to speak — with `rmw_zenoh` running,
this board is another node in the graph.

```
            ┌───────────────────────── Link101 (RP2350A) ─────────────────────┐
            │                                                                  │
 USB CDC #0 │  zenoh ── ROS node "axon"                                        │
◄──────────►│    sub  base_cmd  ──► 4x DDSM210, one PIO UART each              │──► wheels
            │    sub  arm_cmd   ──► Feetech servos on the half-duplex bus      │──► arm
            │    pub  joint_states, motor_telemetry/*                          │
            │    pub  imu/data, imu/mag, imu/temperature ◄── onboard IMU (i2c1)│
 USB CDC #1 │                                                                  │
◄──────────►│  passthrough ◄──────────────────────────────────► uart1          │──► RPLidar C1
 USB CDC #2 │                                                                  │
◄──────────►│  boot log (goes quiet once zenoh is up)                          │
            └──────────────────────────────────────────────────────────────────┘
```

## Where to start

**[`robot.h`](robot.h) is the robot.** Pins, motor IDs, joint names,
directions, rates, topic names, speed limits — all of it, in one file, with
nothing else in the firmware carrying numbers like these. Retuning or
rewiring means editing that file and reflashing.

The rest is one file per thing, ~100 lines each, readable in any order:

| File | What it is |
|---|---|
| [`main.c`](main.c) | `setup()` then `loop()`. The whole shape of the firmware. |
| [`wheels.c`](wheels.c) | The four drive wheels: speed in, angle out. |
| [`servos.c`](servos.c) | The arm: angle in, angle and telemetry out. |
| [`imu.c`](imu.c) | The onboard LSM6DSOX + MMC5983MA. |
| [`lidar.c`](lidar.c) | Bytes between USB CDC #1 and uart1. |
| [`ros.c`](ros.c) | Publishers, subscribers, and what goes out when. |
| [`status.c`](status.c) | The boot log, and what the LED strip is saying. |
| [`io.c`](io.c) | USB, and the heartbeat every blocking wait runs. |

Everything below that — the board, the drivers, zenoh, the ROS layer — is a
library, in `lib/` as a submodule. See [Libraries](#libraries).

## ROS interface

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| sub | `/motor_manager/base_cmd` | `std_msgs/Float64MultiArray` | 4 wheel speeds, rad/s |
| sub | `/motor_manager/arm_cmd` | `std_msgs/Float64MultiArray` | arm joint angles, rad |
| pub | `/motor_manager/joint_states` | `sensor_msgs/JointState` | wheels + arm, 50 Hz |
| pub | `/motor_telemetry/<joint>/current` | `std_msgs/Float32` | servo current, mA |
| pub | `/motor_telemetry/<joint>/voltage` | `std_msgs/Float32` | servo voltage, V |
| pub | `/motor_telemetry/<joint>/load` | `std_msgs/Float32` | servo load, % |
| pub | `/motor_telemetry/<joint>/temperature` | `std_msgs/Int32` | servo temperature, °C |
| pub | `/imu/data` | `sensor_msgs/Imu` | angular velocity + acceleration, 50 Hz |
| pub | `/imu/mag` | `sensor_msgs/MagneticField` | magnetometer, tesla |
| pub | `/imu/temperature` | `sensor_msgs/Temperature` | IMU die temperature, °C |

Command arrays are in the order of the `WHEELS` and `SERVOS` tables in
`robot.h`; reorder a table and both the command array and the joint_states
slots follow. Wheel speeds are `rad/s × direction`, capped at
`WHEEL_MAX_RPM`; arm angles are radians from centre. Repeating a command is
free — an unchanged value is not re-sent to the bus.

Worth knowing:

- **Wheel speed changes are ramped, not instant.** A `base_cmd` step --
  reversing direction, or a turn asking the two sides for very different
  speeds -- doesn't reach the motors as a jump. The actual commanded speed
  moves toward whatever was last asked for at up to
  `WHEEL_ACCEL_LIMIT_RAD_S2` (robot.h) rad/s², so the wheel can follow it
  without skidding across the floor getting there. Lower that constant for
  a gentler stop/turn, raise it to track the host more closely.
- **A stale `base_cmd` brakes, hard, not ramped.** If nothing arrives for
  `COMMAND_TIMEOUT_MS`, every wheel gets the DDSM210's active brake
  immediately (not the gentle ramp above) — a host that stalls, crashes,
  or drops the ROS link should not leave the robot coasting on the last
  thing it heard.
- **Stamps are time since boot.** The board has no RTC and nothing to sync
  one against. Re-stamp on the host if it matters.
- **Every joint appears every time.** A motor that doesn't answer reports
  zero rather than dropping out, so the slots the host reads never shift.
- **Missing hardware is not fatal.** Whatever doesn't answer at boot is
  logged and skipped; its topics still exist and simply stay empty. No fake
  data is ever published.
- **`/imu/data` carries no orientation.** The onboard IMU is a raw 6-axis
  sensor with no fusion engine, so the message sets
  `orientation_covariance[0] = -1` — the `sensor_msgs/Imu` way of saying the
  quaternion is meaningless — and leaves the quaternion zeroed. Run
  `imu_filter_madgwick` or `robot_localization` on the host against
  `/imu/data` + `/imu/mag` if you need attitude. (The previous firmware
  published a fused quaternion because it used a BNO055 on the Qwiic
  connector, which does fusion on-chip.)
- `linear_acceleration` includes gravity, per the `sensor_msgs/Imu`
  convention. Covariances are the fixed nominal diagonals in `robot.h` —
  neither chip reports per-axis variance.

## Hardware

| Bus | Pins | Notes |
|---|---|---|
| Servo bus | TX 7, RX 8, TXEN 16 | Feetech STS/SCS, half duplex, 1 Mbaud. Board-fixed. |
| Wheels | GP19–26 | Four PIO UARTs, 115200. One motor per port — a DDSM210 can't share a TX line. |
| Lidar | TX 4, RX 5 | hardware `uart1`, 460800 (RPLidar C1) |
| IMU | SDA 14, SCL 15 | `i2c1`: LSM6DSOX at 0x6B, MMC5983MA at 0x30. Both soldered to the board; the Qwiic connector is the same bus. |
| LED strip | GP18 | six WS2812 pixels |

PIO state machines are claimed at init, not assigned by hand: 2 for the
servo bus, 8 for the wheels, 1 for the LEDs — 11 of the 12 the RP2350 has.
If a port can't get one, `begin()` says so in the boot log instead of
failing strangely later.

USB (VID:PID `1209:AC01`) presents three CDC ports: `RoboCore Axon Zenoh`,
`RoboCore Axon Lidar`, `RoboCore Axon Debug`. The udev rules below turn
those into stable names.

## Building

Needs the [pico-sdk](https://github.com/raspberrypi/pico-sdk) 2.x and an
`arm-none-eabi` toolchain.

```bash
git submodule update --init          # the libraries in lib/
export PICO_SDK_PATH=~/pico/pico-sdk
cmake -B build -DCMAKE_BUILD_TYPE=Release && make -C build -j$(nproc)
```

Flash by holding BOOT while plugging in and copying `build/base101_firmware.uf2`
to the `RPI-RP2` drive, or `picotool load -f build/base101_firmware.uf2`.

The boot log is on USB CDC #2 (`screen /dev/axon-debug 115200`) and tells you
what answered:

```
=== base101 firmware ===
[boot ] USB up: CDC0 zenoh, CDC1 lidar, CDC2 this log
[lidar] uart1 on GP4/5 at 460800 baud
[wheel] front_left_wheel_joint on GP21/22: online
...
[ros  ] connecting to the router on 'serial/cdc#baudrate=921600'...
[ros  ] session up; declaring 'axon'
[boot ] up. Going quiet -- watch the LED, or the ROS graph.
```

It stops there on purpose: in steady state every byte of USB bandwidth
belongs to the zenoh transport. **The LED takes over from there:**

| Strip | Means |
|---|---|
| fast yellow blink | waiting for the ROS router — nothing else can start |
| slow green breath | connected and running |
| frozen | the main loop stopped turning |

It is driven from the main loop and from inside every blocking wait, so it
keeps moving even while the board waits for a router that isn't up yet.
Colours and rates are in `robot.h`.

## Host setup

### 1. udev rules

```bash
./install.sh
```

gives you stable symlinks:

- `/dev/axon-zenoh` — zenoh serial transport
- `/dev/axon-lidar` — lidar passthrough
- `/dev/axon-debug` — boot log

### 2. zenoh router with serial transport

The firmware is a zenoh **client** connecting through its serial link, so the
host must run a router listening on that port. Serial transport is **not
enabled in stock zenohd builds** — you need one compiled with
`--features transport_serial`.

Easiest is the Docker image in [`docker/`](docker/), which rebuilds zenohd
with that feature on top of `eclipse/zenoh:latest`:

```bash
cd docker
docker compose up --build      # maps /dev/axon-zenoh, listens serial + tcp/7447
```

From source instead:

```bash
git clone https://github.com/eclipse-zenoh/zenoh && cd zenoh
cargo build --release -p zenohd --features transport_serial
# the baudrate token is required by the locator syntax, and meaningless on USB CDC
./target/release/zenohd -l 'serial//dev/axon-zenoh#baudrate=921600'
```

Keep the router protocol-compatible with the vendored zenoh-pico (currently
**1.9.0**, see `lib/zenoh-pico/include/zenoh-pico.h`).

### 3. ROS 2 with rmw_zenoh

```bash
# option A: single router — also listen on the tcp port rmw_zenoh expects
zenohd -l 'serial//dev/axon-zenoh#baudrate=921600' -l 'tcp/[::]:7447'

# option B: keep rmw_zenohd, and federate the serial router into it
zenohd -l 'serial//dev/axon-zenoh#baudrate=921600' -e 'tcp/localhost:7447'
```

Then, with `RMW_IMPLEMENTATION=rmw_zenoh_cpp`:

```bash
ros2 topic list
ros2 topic echo /motor_manager/joint_states
# all four wheels at 1 rad/s: [front_left, front_right, back_left, back_right]
ros2 topic pub -r 20 /motor_manager/base_cmd std_msgs/msg/Float64MultiArray '{data: [1.0, 1.0, 1.0, 1.0]}'
# arm to home
ros2 topic pub --once /motor_manager/arm_cmd std_msgs/msg/Float64MultiArray '{data: [0, 0, 0, 0, 0, 0]}'
```

### 4. Lidar

Point `rplidar_ros` at `/dev/axon-lidar` at 460800. The firmware copies bytes
both ways and follows a baud rate change made on the port, so the driver
behaves as if the lidar were plugged into the host.

## Libraries

Everything below the robot is a library, pulled in as a submodule under
`lib/`:

| Library | What it does |
|---|---|
| [`pico_serial`](https://github.com/robocore-labs/pico_serial) | The `serial_t` interface every driver speaks, and `serial_hook.h`. |
| [`pico_cdc_serial`](https://github.com/robocore-labs/pico_cdc_serial) | A USB CDC interface as a `serial_t`. The one place USB stops. |
| [`hardware_link101`](https://github.com/robocore-labs/hardware_link101) | The board: PIO and UART ports, pin map, LED strip, CAN, and the onboard LSM6DSOX + MMC5983MA. |
| [`pico_feetech`](https://github.com/robocore-labs/pico_feetech) | Feetech STS/SCS servos. |
| [`pico_ddsm`](https://github.com/robocore-labs/pico_ddsm) | DDSM210 wheel motors. |
| [`pico_zenoh`](https://github.com/robocore-labs/pico_zenoh) | zenoh-pico for bare metal, over any `serial_t`. |
| [`easypicoros`](https://github.com/robocore-labs/easyp) | Typed ROS publishers and subscribers on top of Pico-ROS. |

zenoh-pico, picoros and micro-CDR stay vendored in `lib/` rather than
submoduled: they are upstream projects we track, and picoros carries a local
patch (a `user_data` pointer on `picoros_subscriber_t`, without which a typed
subscriber facade is impossible — easypicoros' build checks for it).

### How the pieces fit

Every bus in the firmware is a `serial_t`, and the drivers take one. So a
wheel motor, a servo, the lidar and the zenoh link are all the same kind of
thing, and nothing below the application knows what USB is.

The interesting part is **`io_poll()`**. Drivers block waiting for a reply and
call `serial_task()` while they wait, so every bus is wrapped with
`serial_hook_init()` to run `io_poll()` there: USB, the lidar bridge and the
LED keep going *inside* a servo read, a wheel read, or zenoh waiting for a
router that isn't up yet. One hook, everywhere, with one rule — `io_poll()`
must never touch a wrapped bus or call into zenoh, or it would be calling
itself.

## What changed, and why

This firmware used to carry its own copies of everything: two near-identical
PIO UART implementations, the motor drivers, the IMU driver, the zenoh port,
its own ROS message catalogue. All of it now lives in the libraries above,
shared with the other Link101 firmwares — around 950 lines that were here are
gone, and the drivers gained things the old copies never had (SYNC reads for
whole-chain servo transactions, multiple IMUs per bus).

**Config mode is gone.** The board used to boot into a JSON console on the
debug port — held the button at boot, edited the servo list, saved it to
flash. It was worth having while bringing the hardware up, and nothing used
it afterwards. The servo list is compiled in (`SERVOS` in `robot.h`), and
with it went the flash persistence, the console, the bench-test commands, the
config web page and the button check at boot.

The old firmware is kept at
`../attic/firmware-before-link101-libs/` if you need to look something up.
