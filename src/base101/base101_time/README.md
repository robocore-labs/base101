# base101_time

Host-side responder for the Axon 2 firmware's clock-sync probes. The
firmware needs the host<->board offset to put a correct host-clock stamp on
`/imu/*`, so it probes the host at 1 Hz and does the offset math itself; this
node's only job is to answer honestly and quickly.

| Direction | Topic | Type |
|---|---|---|
| Subscribe | `/link101/time_sync/request` | `std_msgs/msg/UInt64` — board's monotonic transmit time, microseconds |
| Publish | `/link101/time_sync/response` | `std_msgs/msg/Int64MultiArray` — `[echoed_board_transmit_us, host_receive_ns, host_send_ns]` |

Both endpoints use reliable, volatile QoS with depth 1.

## Behavior

- No timer: purely reactive, one response per request.
- Timestamps come from an explicit `ClockType.SYSTEM_TIME` clock, never the
  node's regular (possibly sim-time) clock — the firmware's offset estimate
  has to be against the real wall clock regardless of what else is going on
  in the launch tree.
- `host_receive_ns` is captured on callback entry, `host_send_ns` immediately
  before publishing, so the reported host-side interval matches the work
  actually done in between.
- The firmware rejects round trips over 20 ms, so this callback does nothing
  beyond the two clock reads, the copy, and the publish — don't add work
  here, and don't run this node on an executor shared with anything that
  blocks.
- A request whose value would overflow `int64` is dropped (logged, not
  answered) rather than echoed back corrupted, since the response fields are
  signed.

## Running

Launch exactly one instance per board — it's brought up unconditionally by
`base101_bringup_hw/launch/robot.launch.py`, in the same ROS domain and zenoh
network as the firmware. The host's own system clock is assumed to already
be correct (NTP or equivalent) — this node reports it, it doesn't discipline
it.

```bash
ros2 run base101_time time_sync
```
