# ptc_cli — build and bench test

Interactive PTC test tool for JT128 lidar over TCP port 9347.

## Build

```bash
cd ~/HesaiLidar_SDK_2.0/tool_ptc
mkdir -p build && cd build
cmake .. && cmake --build . -j$(nproc)
```

## Run

```bash
./ptc_cli 192.168.1.202 9347   # left lidar (typical post-RDK)
./ptc_cli 192.168.1.201 9347   # right lidar or factory IP
```

Usage: `ptc_cli <ip> <port>`

---

## How to test (GET → SET → GET)

For every **writable** setting, use the same pattern:

1. **GET** — read current value (note baseline)
2. **SET** — write a test value
3. **GET again** — confirm readback **MATCH** (CLI prints MATCH/MISMATCH where supported)
or manually run get again.

Use **menu 1 `read_all`** anytime for a quick snapshot (device IP, dest UDP port, return mode, spin rate, PTP offset, point cloud, PTP status).

---

## Menu reference

```
  GET
    1  read_all
    2  get_return_mode
    3  get_spin_rate
    4  get_ptp_lock_offset
    5  get_point_cloud_config (GET 0x122 live ultra + filter)
    6  get_config_info_raw (len + head hex — debug)
    7  get_ptp_diagnostics
    8  get_lidar_status
    9  get_calibration (size only)
  SET
   10  set_return_mode
   11  set_spin_speed (600 or 1200 RPM only)
   12  set_ptp_lock_offset (1–1000 us, e.g. 350)
   13  set_point_cloud (ultra + filter, or 'none' to keep)
   14  set_filter_only
   15  set_ultra_precise_only
    0  quit
```

---

## Notes

- **PTP diagnostics (7)** may fail with `ret_code=5` while status is **Free run (0)**; that is expected.

---

## Network SET test (`ptc_network_test`)

**WARNING: one lidar only.** Writes destination UDP + device IP (RDK-style). Requires power cycle.

```bash
./ptc_network_test <current_ip> <new_device_ip> <dest_udp_port> [ptc_tcp_port]

# RDK left example (factory IP → .202, UDP 2378):
./ptc_network_test 192.168.1.201 192.168.1.202 2378
```

Flow: GET config at `current_ip` → SET dest (`255.255.255.255`, UDP port) → SET net (new IP, Stretch mask/gateway) → prompt power cycle → reconnect at `new_device_ip:9347` → GET and PASS/FAIL device IP + dest UDP.
