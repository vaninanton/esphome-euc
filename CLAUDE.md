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

Entry-point configs, each a different device combination:

| File | Devices |
|---|---|
| `esphome-euc-lynx-charger.yaml` | Veteran Lynx + Fast charger |
| `esphome-euc-nosfet-charger.yaml` | Nosfet Aero + Fast charger |
| `esphome-euc-lynx-nosfet-charger.yaml` | Veteran Lynx + Nosfet Aero + Nosfet Xeno + Fast charger |
| `esphome-euc-lynx-nosfet-charger-c6.yaml` | same, for XIAO ESP32C6 board |

Each config composes reusable packages via `!include`:
- `package-veteran-device.yaml` — one Veteran/NOSFET wheel (parameterised via vars)
- `package-charger-device.yaml` — one Fast charger (parameterised via vars)

The entry-point YAML owns: `esphome:` devices list, `web_server:` sorting_groups, and the `switch: Bluetooth` / `binary_sensor: any_ble_connected` lambdas that reference all active device IDs by name.

## Custom Component: `veteran`

ESPHome custom component for Veteran/NOSFET BLE protocol. **Lives in a separate repo** —
`github://vaninanton/esphome-euc-leaperkim-nosfet` (`components/veteran/`), pulled in via
`external_components`. To iterate locally, temporarily point `source:` at
`../esphome-euc-leaperkim-nosfet/components` and revert before committing. Same arrangement for
the `charger` component (`github://vaninanton/esphome-euc-hw-smart-charger`).

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
| `euc_nominal_voltage` | `151.2` | Full-charge voltage, i.e. cells × 4.2 — NOT the nominal. 36S=151.2, 30S=126 |
| `euc_cell_count` | `36` | For BMS min/max/delta calculation |
| `euc_charge_voltage_min` | `147.0` | Lower bound for `Max charging voltage` number |
| `euc_charge_voltage_offset` | `145` | `charge_voltage_base`: Lynx=145, Aero/Xeno=121 |

`package-charger-device.yaml` vars: `charger_id`, `charger_mac`, `charger_device_id`.

`euc_charge_voltage_offset` is used in **both** directions and needs no empirical calibration:
outgoing `set_max_charge` encodes `byte24 = clamp((volts - base) * 10, 0, 120)`, and the incoming
value decodes as `base * 10 + packet[64]`, published as `/10`. Byte 64 is the same byte the command
writes. For Leaperkim wheels the base also arrives in Settings `[65]` (`0` means 145); Nosfet
firmware puts something else there, so it stays configured by hand.

A `euc_charge_stop_offset` var used to exist for decoding, with per-wheel values found by trial
(Lynx 682, Aero −70, Xeno 442). It was an artifact: the old formula read a `u16` at offset 63,
whose high byte is the key-tone volume, and each offset was exactly `base * 10 - packet[63] * 256`.
Removed — do not reintroduce it.

## ESP32-C6 BLE gotcha

On the C6 board, `esp32_ble` needs an sdkconfig override or only ONE BLE connection will ever establish:

```yaml
sdkconfig_options:
  CONFIG_BT_BLE_50_FEATURES_SUPPORTED: n
  CONFIG_BT_LE_MAX_CONNECTIONS: "4"
```

- **BLE 4.2 vs 5.0** — ESPHome forces `CONFIG_BT_BLE_42_FEATURES_SUPPORTED=y`, while on C6 (`SOC_BLE_50_SUPPORTED`)
  `CONFIG_BT_BLE_50_FEATURES_SUPPORTED` defaults to `y`. IDF's Kconfig states outright that
  "BLE 4.2 and BLE 5.0 cannot be used simultaneously". With both on, Bluedroid issues LE Extended Create
  Connection (HCI `0x2043`), the controller rejects it as `Cmd Disallowed`, and every client after the first
  fails with `status=133`.
- **Connection slots** — ESPHome maps `max_connections` onto `CONFIG_BTDM_CTRL_BLE_MAX_CONN`, which is the
  *classic ESP32* controller option. C6 uses `CONFIG_BT_LE_MAX_CONNECTIONS`, left at its default 3.

Neither applies to the plain `esp32dev` configs: classic ESP32 has no BLE 5.0 at all, and its controller option
is the one ESPHome already sets.

## Secrets

```yaml
wifi_ssid / wifi_password
ha_encryption_key
euc_veteran_mac / euc_nosfet_mac / euc_xeno_mac
fast_charger_mac
```

## Protocol Docs

Both moved into their component repos:

- `esphome-euc-leaperkim-nosfet/docs/veteran-protocol.md` — BLE packet layout for Veteran/NOSFET,
  outgoing command opcodes, protocol-version to model map
- `esphome-euc-hw-smart-charger/docs/CHARGER.md` — BLE protocol for the Fast charger (FFE1 service)
- `docs/inmotion-protocol.md` — InMotion notes (this repo)
