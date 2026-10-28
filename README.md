# mav_ardupilot

Transparent MAVLink-over-WiFi bridge on two Seeed XIAO ESP32S3 boards
(ESP-IDF). The pair replaces a set of telemetry radios:

```
laptop --USB-- [GROUND: WiFi AP] ~~WiFi / UDP 14550~~ [DRONE: WiFi STA] --UART 115200-- flight controller
```

Bytes are relayed as-is (MAVLink is not parsed), so QGroundControl,
MAVProxy or pymavlink talk to the FC as if it were plugged in directly.
Both boards run the same firmware; the role is chosen at build time.

- **GROUND** — WiFi access point (`mavbridge`, channel 1, WPA2,
  192.168.4.1, at most 1 client). MAVLink goes over the board's native USB
  (USB-Serial-JTAG, shows up as `/dev/ttyACM*`; baud rate is ignored).
  Logs go to UART0 so the USB port carries nothing but MAVLink.
- **DRONE** — joins the AP and sends UDP to 192.168.4.1:14550. MAVLink goes
  over UART1 to the FC; UDP packets from any address other than the ground
  board are dropped. Logs go to USB, including a `FC -> N bytes in the last
  2 s` line every 2 s that confirms the FC wiring.

## Wiring (drone side)

| XIAO ESP32S3      | FC (Kakute H7 Mini, UART4) |
|-------------------|----------------------------|
| D6 / GPIO43 (TX)  | R4                         |
| D7 / GPIO44 (RX)  | T4                         |
| GND               | G                          |
| 5V                | 5V pad (powered only when the battery is plugged in) |

Pins and baud rate are `#define`d at the top of [main/main.c](main/main.c)
(`PIN_TX`, `PIN_RX`, `MAV_BAUD`). The ground board needs only USB.

## FC configuration

For the UART used above (SERIAL4 on the Kakute H7 Mini):

- `SERIAL4_PROTOCOL = 2` (MAVLink2)
- `SERIAL4_BAUD = 115` (115200, must match `MAV_BAUD`)

Reboot the FC after changing these.

## Building

Requires ESP-IDF v5.5.x (tested with v5.5.2) targeting `esp32s3`, and the MAVLink submodule
(`git clone --recurse-submodules`, or `git submodule update --init
--recursive`).

Each role has its own build directory and its own generated `sdkconfig`,
seeded from `sdkconfig.<role>.defaults`. `sdkconfig*` and `build*/` are not
tracked, so a plain `idf.py build` is **not** the way to build: it would
produce a default (DRONE) config.

```bash
source ~/esp/esp-idf/export.sh

idf.py -B build-ground -D SDKCONFIG=sdkconfig.ground -D SDKCONFIG_DEFAULTS=sdkconfig.ground.defaults -D IDF_TARGET=esp32s3 build
idf.py -B build-drone  -D SDKCONFIG=sdkconfig.drone  -D SDKCONFIG_DEFAULTS=sdkconfig.drone.defaults  -D IDF_TARGET=esp32s3 build
```

The defaults files are only applied when the generated `sdkconfig.<role>`
does not exist yet; delete it after editing a `.defaults` file.

## Flashing

The boards are identical, so use the stable `/dev/serial/by-id/` path to
pick the right one:

```bash
idf.py -B build-ground -D SDKCONFIG=sdkconfig.ground -p /dev/serial/by-id/<ground board> flash
idf.py -B build-drone  -D SDKCONFIG=sdkconfig.drone  -p /dev/serial/by-id/<drone board>  flash monitor
```

If the board stays in `waiting for download` after flashing, press its
RESET button once.

## Using it

```bash
# MAVProxy on the ground board, fanned out to QGC (14550) and a script (14551)
mavproxy.py --master=$(readlink -f /dev/serial/by-id/<ground board>) --baudrate 115200 \
            --out=udp:127.0.0.1:14550 --out=udp:127.0.0.1:14551 --streamrate=-1
```

pymavlink and MAVProxy treat a `:` in the device name as `host:port`, and the
Espressif by-id names contain the MAC address, hence `readlink -f`.
`--streamrate=-1` stops MAVProxy from sending `REQUEST_DATA_STREAM`, which
ArduPilot stores in the `SRn_*` parameters; clients request their own
streams with `SET_MESSAGE_INTERVAL` instead.

## WiFi password

The WPA2 key is `CONFIG_MAV_WIFI_PASS` in **both** `sdkconfig.*.defaults`
files and must be identical in both. The Kconfig default (`password`) is
only a fallback. To change it, edit both files, delete `sdkconfig.ground`
and `sdkconfig.drone`, rebuild and reflash both boards.
