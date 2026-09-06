# mav_ardupilot

ESP32-S3 &lt;-&gt; ArduPilot MAVLink2 bridge, built on ESP-IDF. The ESP32 acts as a
standalone companion computer wired directly to a flight controller's TELEM
port: it keeps the link alive with a 1 Hz `HEARTBEAT`, locks onto the FC's
sysid/compid, and drives the FC through a hardcoded list of MAVLink commands
defined in [main/main.c](main/main.c).

## Hardware

- ESP32-S3 dev board.
- ArduPilot flight controller with a free `SERIALn` (TELEM) port.
- Wiring (crossed, common ground):

  | ESP32-S3      | FC TELEM port |
  |---------------|---------------|
  | GPIO17 (TX)   | RX            |
  | GPIO18 (RX)   | TX            |
  | GND           | GND           |

  Pins are `#define`d at the top of `main.c` (`PIN_TX`/`PIN_RX`) if your board
  needs different ones.

## FC-side configuration

On the `SERIALn` port you wired up (e.g. TELEM2 = `SERIAL2` on most boards),
via Mission Planner / QGroundControl:

- `SERIAL2_PROTOCOL = 2` (MAVLink2)
- `SERIAL2_BAUD = 115` (115200, must match `MAV_BAUD` in `main.c`)
- `SERIAL2_OPTIONS = 0` (no inversion/swap/half-duplex needed for a normal
  straight wiring)

Reboot the FC after changing these.

## Building

Requires [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) (developed
against v5.5.x) targeting `esp32s3`.

```bash
git clone --recurse-submodules <this repo's URL>
cd mav_ardupilot
source $IDF_PATH/export.sh
idf.py build
```

If you already cloned without `--recurse-submodules`, run
`git submodule update --init --recursive` first — the MAVLink C headers
(`components/mavlink/c_library_v2`) are a submodule and the build will fail
without them.

## Flashing

```bash
idf.py -p /dev/ttyACM0 flash monitor
```

Adjust the port (`/dev/ttyACM0`/`/dev/ttyACM1` on Linux depending on your
board). If the monitor gets stuck showing `waiting for download` after
flashing, press the board's **RESET/EN** button once — this is a known
ESP32-S3 quirk where the auto-reset sequence sometimes leaves the chip
latched in USB download mode instead of booting the app.

## What it does

1. Sends `HEARTBEAT` at 1 Hz so the FC treats the link as alive.
2. Locks onto the FC's sysid/compid from its own `HEARTBEAT` (compid
   `MAV_COMP_ID_AUTOPILOT1` only, ignoring other GCS/companions that might
   share the bus).
3. Once locked, walks a hardcoded list of commands (`g_commands[]`), one
   every 2 seconds, and logs each resulting `COMMAND_ACK`.
4. Decodes and logs a few message types it's specifically interested in
   (`AUTOPILOT_VERSION`, `SYS_STATUS`, `ATTITUDE`) as they arrive.

### Example: pulling live attitude + flight mode from the FC

`g_commands[]` includes an entry that asks the FC to stream `ATTITUDE`
(roll/pitch/yaw + a boot-relative timestamp) continuously at 1 Hz, using
`MAV_CMD_SET_MESSAGE_INTERVAL` — the "subscribe" version of the one-shot
`MAV_CMD_REQUEST_MESSAGE`:

```c
{ MAV_CMD_SET_MESSAGE_INTERVAL, { MAVLINK_MSG_ID_ATTITUDE, 1000000, 0, 0, 0, 0, 0 } },
```

`param1` is the message ID to stream, `param2` the interval in microseconds
(1,000,000 = 1 Hz). Every `ATTITUDE` message that arrives afterward is
decoded and logged alongside the current flight mode, which is tracked
separately from every `HEARTBEAT.custom_mode`:

```c
case MAVLINK_MSG_ID_ATTITUDE: {
    mavlink_attitude_t att;
    mavlink_msg_attitude_decode(msg, &att);
    ESP_LOGI(TAG, "ATTITUDE t=%lu ms roll=%.1f pitch=%.1f yaw=%.1f deg mode=%s",
             (unsigned long)att.time_boot_ms,
             att.roll  * (180.0f / (float)M_PI),
             att.pitch * (180.0f / (float)M_PI),
             att.yaw   * (180.0f / (float)M_PI),
             copter_mode_name(g_flight_mode));
    break;
}
```

Expected log output once the FC accepts the stream request:

```
I (xxx) mav: ATTITUDE t=12345 ms roll=0.3 pitch=-1.2 yaw=88.4 deg mode=STABILIZE
I (xxx) mav: ATTITUDE t=13345 ms roll=0.2 pitch=-1.1 yaw=88.6 deg mode=STABILIZE
```

`copter_mode_name()` maps `HEARTBEAT.custom_mode` to a human-readable
ArduCopter flight mode name (`STABILIZE`, `GUIDED`, `RTL`, ...) using the
`COPTER_MODE` enum from the `ardupilotmega` dialect — swap in the
`PLANE_MODE`/`ROVER_MODE` enum instead if you're bridging a different
vehicle type.

## Adding your own commands

Edit `g_commands[]` in `main.c` — each entry is a `MAV_CMD_*` plus its 7
`COMMAND_LONG` params:

```c
typedef struct {
    uint16_t cmd;
    float    param[7];
} mav_cmd_t;
```

The list is sent once, in order, 2 seconds apart, starting as soon as the FC
is seen. A couple of useful ones to try:

- `{ MAV_CMD_DO_SET_MODE, { MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, <custom_mode>, 0,0,0,0,0 } }` — change flight mode.
- `{ MAV_CMD_COMPONENT_ARM_DISARM, { 1, 0,0,0,0,0,0 } }` — arm the vehicle.
  **Only test this with propellers removed or on a bench** — it will
  genuinely arm the FC.

## Suggested workflow for new behavior

Since iterating on real C firmware (edit → build → flash → reset) is slow,
prototype new command sequences first with
[pymavlink](https://github.com/ArduPilot/pymavlink) directly against the FC
from a PC (no ESP32 involved):

```python
from pymavlink import mavutil
m = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
m.wait_heartbeat()
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 0, 0, 0, 0, 0, 0)
print(m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3))
```

Once a sequence is proven, port it into `g_commands[]` (or a new
`handle_message()` case, for messages you need to react to) and flash it —
the deployed ESP32 then runs standalone with no PC/GCS attached.
