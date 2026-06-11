# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

```bash
# Validate config without building
esphome config <file>.yaml

# Build and flash via USB
esphome run <file>.yaml

# Build only (no flash)
esphome compile <file>.yaml

# Stream logs from running device
esphome logs <file>.yaml

# Find BLE MAC addresses
esphome run esphome-euc-ble-mac-scanner.yaml
```

Always run `esphome config <file>.yaml` after editing any YAML to catch validation errors before flashing.

## Project Structure

Three entry-point configs, each a different device combination:

| File | Devices |
|---|---|
| `esphome-lynx-charger.yaml` | Veteran Lynx + Fast charger |
| `esphome-nosfet-charger.yaml` | Nosfet Aero + Fast charger |
| `esphome-lynx-nosfet-charger.yaml` | Veteran Lynx + Nosfet Aero + Fast charger |

Each config composes reusable packages via `!include`:
- `package-veteran-device.yaml` — one Veteran/NOSFET wheel (parameterised via vars)
- `package-charger-device.yaml` — one Fast charger (parameterised via vars)

The entry-point YAML owns: `esphome:` devices list, `web_server:` sorting_groups, and the `switch: Bluetooth` / `binary_sensor: any_ble_connected` lambdas that reference all active device IDs by name.

## Custom Component: `my_components/veteran/`

ESPHome custom component for Veteran/NOSFET BLE protocol.

**`__init__.py`** — ESPHome codegen. `ENTITY_REGISTRY` is the single source of truth for all sensors/binary_sensors/text_sensors: name, icon, device_class, unit, sorting weight. Adding a new sensor means adding one entry here + the corresponding setter/field in the C++ files.

**`veteran.h` / `veteran.cpp`** — C++ component. Key methods:
- `parse_ble_packet()` — called from YAML lambda on BLE notify; reassembles BLE chunks into full packets using `ble_buffer_`, validates CRC32, queues to `packet_queue_`
- `loop()` — drains `packet_queue_` (one packet per loop call), calls `parse_packet()` → `publish_state_from_euc()` throttled to 2000 ms
- `parse_packet()` — dispatches common payload (offset 4–35, all packets) and extended payload (offset 46+, sub-typed by SubType enum) into `euc_` struct
- `clear_realtime_data()` — called on BLE disconnect; resets `euc_` to `{}` but preserves mileage, tho_ra, low_power_mode, high_speed_mode, firmware version, charging_stop_voltage
- `get_charge_packet(voltage)` — builds BLE command; uses `charge_voltage_offset_` (default 145.0, set per-device via `CONF_CHARGE_VOLTAGE_OFFSET`)

`EUCData euc_` is the central state struct. `publish_state_from_euc()` pushes it to all ESPHome sensors.

Extended payload sub-types (byte 46): `0x00/0x04` Live (controller temp, BMS currents, headlight level), `0x01–0x03` BMS left cells+temps, `0x05–0x07` BMS right cells+temps, `0x08` Settings (headlight, low/high-speed mode, tho_ra, charging_stop_voltage).

42S batteries are theoretically supported: `parse_bms_temps_and_cells()` reads cells 37–42 when packet size ≥ 83 bytes (`BMS_CELLS_37_BASE`). Set `euc_cell_count: 42` in vars.

## Package Variables

`package-veteran-device.yaml` vars:

| Var | Example | Notes |
|---|---|---|
| `euc_id` | `lynx` | Used as C++ ID prefix — no spaces |
| `euc_mac` | `!secret euc_veteran_mac` | |
| `euc_device_id` | `lynx_device` | Must match `esphome: devices:` entry |
| `euc_nominal_voltage` | `151.2` | 36S=151.2, 30S=126 |
| `euc_cell_count` | `36` | For BMS min/max/delta calculation |
| `euc_charge_voltage_min` | `147.0` | Lower bound for `Max charging voltage` number |
| `euc_charge_stop_offset` | `682` | Decode offset for incoming charging_stop_voltage: Lynx=682, Aero=-70 |
| `euc_charge_voltage_offset` | `145` | Base voltage for outgoing set_max_charge command: Lynx=145, Aero=121 |

`package-charger-device.yaml` vars: `charger_id`, `charger_mac`, `charger_device_id`.

## Secrets

```yaml
wifi_ssid / wifi_password
ha_encryption_key
euc_veteran_mac / euc_nosfet_mac
fast_charger_mac
```

## Protocol Docs

- `docs/veteran-protocol.md` — full BLE packet layout for Veteran/NOSFET
- `docs/CHARGER.md` — BLE protocol for Fast charger (FFE1 service)
