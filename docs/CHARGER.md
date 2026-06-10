# Charger BLE Protocol Documentation

## Overview

BLE протокол для управления зарядным устройством (Fast charger) через GATT характеристики.

## Connection Details

- **Handle**: 0x0006 (Write Command target)
- **Notification Handle**: 0x0003 (Status notifications)
- **Connection Type**: ATT Write Command

## Command Structure

### Voltage Setting Command

**Opcode**: `06 07`

**Format**:

```
06 07 [4 bytes float LE] [1 byte checksum]
```

**Parameters**:

- Bytes 0-1: `06 07` — command identifier
- Bytes 2-5: IEEE754 float (little-endian) — target voltage in volts
- Byte 6: Checksum/validation byte

**Response**:
Device acknowledges with: `03 07 01 08`

### Current Setting Command

**Opcode**: `06 08`

**Format**:

```
06 08 [4 bytes float LE] [1 byte checksum]
```

**Parameters**:

- Bytes 0-1: `06 08` — command identifier
- Bytes 2-5: IEEE754 float (little-endian) — current limit in amperes
- Byte 6: Checksum/validation byte

**Response**:
Device acknowledges with: `03 08 01 09`

### Two-Stage Voltage Command

**Opcode**: `06 0A` (estimated)

**Format**:

```
06 0A [4 bytes float LE] [1 byte checksum]
```

**Parameters**:

- Bytes 0-1: `06 0A` — two-stage voltage setting
- Bytes 2-5: IEEE754 float (little-endian) — second-stage target voltage in volts
- Byte 6: Checksum/validation byte

**Known Value**: 151.2V (from app settings screen)

### Two-Stage Current Setting

**Opcode**: unknown (not `06 0B`)

**Notes**:

- `06 0B` is confirmed in your dump as a boolean toggle command, not as a float setting.
- The app parameter `二段电流 2.0A` remains to be mapped to a different opcode.

### Toggle Commands (Toggle/Boolean Settings)

Based on app interface structure and the dump data, toggle commands are confirmed to use 1-byte boolean payloads:

**Format** (confirmed for some toggles):

```
06 [opcode] 01 00 00 00 [checksum]   ; ON
06 [opcode] 00 00 00 00 [checksum]   ; OFF
```

Confirmed toggle opcodes:

- Full Charge Auto-Stop: `06 0C`
  - ON: `06 0C 01 00 00 00 0D`
  - OFF: `06 0C 00 00 00 00 0C`
  - Ack: `03 0C 01 0D`
- Boolean toggle: `06 0B`
  - ON: `06 0B 01 00 00 00 0C`
  - OFF: `06 0B 00 00 00 00 0B`
  - Ack: `03 0B 01 0C`
- Boolean toggle: `06 14`
  - ON: `06 14 01 00 00 00 15`
  - OFF: `06 14 00 00 00 00 14`
  - Ack: `03 14 01 15`

Estimated remaining toggles:

- Power On Output: `06 09` (estimated)
- Two-Stage Switch: `06 0D` (estimated)
- Manual Control: `06 0E` (estimated)

#### Voltage Commands

| Voltage | Hex Bytes              | Details                     |
| ------- | ---------------------- | --------------------------- |
| 126.0V  | `06 07 00 00 FC 42 45` | IEEE754: 0x42FC0000 = 126.0 |
| 127.0V  | `06 07 00 00 FE 42 47` | IEEE754: 0x42FE0000 = 127.0 |
| 151.0V  | `06 07 00 00 17 43 61` | IEEE754: 0x43170000 = 151.0 |
| 151.2V  | `06 07 33 33 17 43 C7` | IEEE754: 0x43173333 = 151.2 |

#### Current Limit Commands

| Current | Hex Bytes              | Details                    |
| ------- | ---------------------- | -------------------------- |
| 1.0A    | `06 08 00 00 80 3F C7` | IEEE754: 0x3F800000 = 1.0  |
| 1.5A    | `06 08 00 00 C0 3F 07` | IEEE754: 0x3FC00000 = 1.5  |
| 3.2A    | `06 08 CD CC 4C 40 2D` | IEEE754: 0x404CCCCD = 3.2  |
| 5.0A    | `06 08 00 00 A0 40 E8` | IEEE754: 0x40A00000 = 5.0  |
| 7.0A    | `06 08 00 00 E0 40 28` | IEEE754: 0x40E00000 = 7.0  |
| 10.0A   | `06 08 00 00 20 41 69` | IEEE754: 0x41200000 = 10.0 |

#### Extended Settings (from App)

| Setting           | Value  | Estimated Opcode | Details                        |
| ----------------- | ------ | ---------------- | ------------------------------ |
| Two-Stage Voltage | 151.2V | `06 0A`          | Second charging stage voltage  |
| Two-Stage Current | 2.0A   | `06 0B`          | Second charging stage current  |
| Shutdown Current  | 0.5A   | `06 0F`          | Current threshold for shutdown |
| Soft Start Time   | 2S     | `06 10`          | Soft-start delay in seconds    |
| Power Limit       | 0W     | `06 11`          | Maximum power (0 = unlimited)  |

### IEEE754 Float Encoding

The voltage is encoded as a 32-bit IEEE754 single-precision float in little-endian byte order:

- Most significant byte last (little-endian)
- Standard IEEE754 representation

Example: 126.0V

```
Float: 126.0
Binary: 01000010 11111100 00000000 00000000
Hex (BE): 42 FC 00 00
Hex (LE): 00 00 FC 42  (transmitted as this in bytes 2-5)
Checksum: 45
```

## Status Query Command

**Opcode**: `02 06`

**Format**:

```
02 06 06
```

**Response**:
Device returns current status and telemetry data with prefix `30 06 ...`

## Device Response

### Voltage Set Acknowledgment

```
03 07 01 08
```

### Status Notification

```
30 06 [2-byte header] [telemetry data...]
```

Pattern: Status packets start with `30 06` followed by ~60 bytes of telemetry data.

## Telemetry Data Structure

Status notifications (opcode `30 06`) contain the following sensor readings:

### Known Fields from Capture

From BLE packet analysis, status responses include (format to be fully decoded):

- Output voltage (Vout): ~151.08V recorded
- Output current (Iout): ~7.0A range
- Input voltage (Vin): ~231V
- Input current (Iin): ~0.00A
- Input frequency: 50Hz
- Temperature readings: 23-24°C
- Efficiency/Power metrics

### Captured Telemetry Sample

```
Raw: 30 06 00 B0 67 43 9D AD 66 3E 00 E1 47 42 00 80 C3 41 00 00 C8 41 DA F0 15 43 00 80 9A 3E FB 8B F3 41 00 A8 AC 42 00 52 74 01 40 C5 EE 96 43 01 A3
```

**Note**: The telemetry format contains multiple IEEE754 float values and other metrics. Exact field mapping requires further reverse engineering of the charger firmware protocol.

### App Display Data

From the control app screenshot:

- **输出电压 (Vout)**: 151.08V
- **输出电流 (Iout)**: 0.00A
- **输入电压 (Vin)**: 231.00V
- **输入电流 (Iin)**: 0.00A
- **输入频率 (Frequency)**: 50Hz
- **温度 (Temperature)**: 23.0°C / 24.0°C
- **充电统计 (Charge)**: 2.03Ah, 302.76Wh
- **当前电压 (Set Voltage)**: 151.2V
- **当前电流 (Set Current)**: 7.0A

## Extended Settings (from App Settings Screen)

Based on the Fast charger app settings interface:

| Parameter             | Chinese  | Current Value | Type   | Notes                                           |
| --------------------- | -------- | ------------- | ------ | ----------------------------------------------- |
| Power On Output       | 开机输出 | Enabled       | Toggle | Controls auto-power-on behavior                 |
| Full Charge Auto-Stop | 充满自停 | Disabled      | Toggle | Auto-stop when fully charged                    |
| Shutdown Current      | 关机电流 | 0.5A          | Float  | Current threshold for shutdown                  |
| Two-Stage Switch      | 二段开关 | Enabled       | Toggle | Enables two-stage charging mode                 |
| Two-Stage Voltage     | 二段电压 | 151.2V        | Float  | Second voltage setpoint                         |
| Two-Stage Current     | 二段电流 | 2.0A          | Float  | Second current setpoint                         |
| Multi-Device Current  | 多机电流 | Smart Control | Enum   | Smart/Manual control for multi-device scenarios |
| Manual Control        | 手动控制 | Enabled       | Toggle | Manual control mode                             |
| Soft Start Time       | 软起时间 | 2S            | Float  | Soft-start delay in seconds                     |
| Power Limit           | 功率限制 | 0W            | Float  | Maximum power output (0 = unlimited)            |
| Language              | 语言     | English       | Enum   | UI language selection                           |

### Toggle Commands (Toggle/Boolean Settings)

Based on app interface structure and the dump data, toggle commands are confirmed to use 1-byte boolean payloads:

**Format** (confirmed for some toggles):

```
06 [opcode] 01 00 00 00 [checksum]   ; ON
06 [opcode] 00 00 00 00 [checksum]   ; OFF
```

Confirmed toggle opcodes:

- Full Charge Auto-Stop: `06 0C`
  - ON: `06 0C 01 00 00 00 0D`
  - OFF: `06 0C 00 00 00 00 0C`
  - Ack: `03 0C 01 0D`
- Boolean toggle: `06 0B`
  - ON: `06 0B 01 00 00 00 0C`
  - OFF: `06 0B 00 00 00 00 0B`
  - Ack: `03 0B 01 0C`
- Boolean toggle: `06 14`
  - ON: `06 14 01 00 00 00 15`
  - OFF: `06 14 00 00 00 00 14`
  - Ack: `03 14 01 15`

Estimated remaining toggles:

- Power On Output: `06 09` (estimated)
- Two-Stage Switch: `06 0D` (estimated)
- Manual Control: `06 0E` (estimated)

### Shutdown Current Command

**Opcode**: `06 0F` (estimated)

**Format**:

```
06 0F [4 bytes float LE] [1 byte checksum]
```

**Known Value**: 0.5A

### Soft Start Time Command

**Opcode**: `06 10` (estimated)

**Format**:

```
06 10 [4 bytes float LE] [1 byte checksum]
```

**Parameters**: Time in seconds (float)

**Known Value**: 2S

### Power Limit Command

**Opcode**: `06 11` (estimated)

**Format**:

```
06 11 [4 bytes float LE] [1 byte checksum]
```

**Parameters**: Power in watts (0 = unlimited)

**Known Value**: 0W

## Protocol Flow

1. **Query Status**: Send `02 06 06`
   - Device responds with `30 06 ...` containing current state

2. **Set Primary Voltage**: Send `06 07 [voltage float LE] [checksum]`
   - Device acknowledges with `03 07 01 08`
   - Voltage is applied

3. **Set Primary Current Limit**: Send `06 08 [current float LE] [checksum]`
   - Device acknowledges with `03 08 01 09`
   - Current limit is applied

4. **Set Two-Stage Voltage** (if enabled): Send `06 0A [voltage float LE] [checksum]`
   - Second charging stage voltage setpoint

5. **Toggle command**: Send `06 0B [00/01] 00 00 [checksum]`
   - Confirmed boolean toggle in the dump; not a float current value
   - Ack: `03 0B 01 0C`

6. **Verify**: Send `02 06 06` again
   - Device responds with updated status

## Checksum Calculation

The checksum byte appears to be calculated from the command bytes. Based on observed patterns:

- The exact algorithm is device-specific
- It validates command integrity
- Different values produce different checksum values

### Known Checksums

#### Voltage Checksums

| Voltage | Last 3 Bytes | Checksum |
| ------- | ------------ | -------- |
| 126.0V  | 00 FC 42     | 45       |
| 127.0V  | 00 FE 42     | 47       |
| 151.0V  | 00 17 43     | 61       |
| 151.2V  | 33 17 43     | C7       |

#### Current Checksums

| Current | Last 3 Bytes | Checksum |
| ------- | ------------ | -------- |
| 1.0A    | 00 80 3F     | C7       |
| 1.5A    | 00 C0 3F     | 07       |
| 3.2A    | CD CC 4C     | 2D       |
| 5.0A    | 00 A0 40     | E8       |
| 7.0A    | 00 E0 40     | 28       |
| 10.0A   | 00 20 41     | 69       |

## Implementation Notes

- All voltage/current commands must use valid IEEE754 float encoding
- The device responds quickly to commands (<100ms typically)
  - Voltage commands: `03 07 01 08` acknowledgment
  - Current commands: `03 08 01 09` acknowledgment
- Status queries return ~200+ bytes of telemetry data
- Commands are confirmed via characteristic notifications
- Multiple consecutive voltage/current changes are supported
- Current and voltage can be set independently

## Testing

Tested voltage values:

- 126.0V ✓
- 127.0V ✓
- 151.0V ✓
- 151.2V ✓

Tested current limits:

- 1.0A ✓
- 1.5A ✓
- 3.2A ✓
- 5.0A ✓
- 7.0A ✓
- 10.0A ✓

All values successfully set and confirmed on device.

### Verification

All commands found and verified in BLE packet traces:

- ✓ Voltage set commands: 126V, 127V, 151V, 151.2V confirmed (opcode 06 07)
- ✓ Current limit commands: 1A, 1.5A, 3.2A, 5A, 7A, 10A confirmed (opcode 06 08)
- ✓ Boolean toggle commands confirmed for `06 0C`, `06 0B`, and `06 14`
- ✓ `06 0B` is confirmed as a toggle ON/OFF command, not a float current command
- ✓ Device acknowledges with appropriate response codes
- ✓ Status notifications show live telemetry updates

## References

- BLE GATT Protocol
- IEEE 754 Float Format
- ATT Write Commands
- Fast charger firmware: tps_2.2.4

## Protocol Coverage

**Primary Commands (Verified via BLE dumps):**

- ✓ Set voltage (4 values: 126V, 127V, 151V, 151.2V)
- ✓ Set primary current (6 values: 1A, 1.5A, 3.2A, 5A, 7A, 10A)
- ✓ Query status (telemetry with ~60 bytes)
- ✓ Voltage acknowledgment (`03 07 01 08`)
- ✓ Current acknowledgment (`03 08 01 09`)

**Extended Commands (Discovered via App UI, opcodes estimated):**

- Two-stage voltage setting (151.2V example found)
- Two-stage current setting (2.0A example found)
- Shutdown current threshold (0.5A)
- Soft start time (2 seconds)
- Power limit (0W = unlimited)
- Power on output (toggle)
- Full charge auto-stop (toggle)
- Two-stage mode switch (toggle)
- Manual control mode (toggle)

**Pending Verification:**

- Exact opcodes for extended commands (0x0A, 0x0B, 0x0C-0x0F estimated)
- Toggle command format and byte structure
- Confirmation codes for extended command responses
- Complete telemetry field mapping in status packets

## Command Summary

| Opcode  | Command Type                   | Response             | Format                           |
| ------- | ------------------------------ | -------------------- | -------------------------------- |
| `06 07` | Set Primary Voltage            | `03 07 01 08`        | `06 07 [float LE] [checksum]`    |
| `06 08` | Set Primary Current Limit      | `03 08 01 09`        | `06 08 [float LE] [checksum]`    |
| `06 0A` | Set Two-Stage Voltage          | `03 0A ?? ??` (est.) | `06 0A [float LE] [checksum]`    |
| `06 0B` | Boolean toggle (confirmed)     | `03 0B 01 0C`        | `06 0B [00/01] 00 00 [checksum]` |
| `06 0F` | Set Shutdown Current           | `03 0F ?? ??` (est.) | `06 0F [float LE] [checksum]`    |
| `06 10` | Set Soft Start Time            | `03 10 ?? ??` (est.) | `06 10 [float LE] [checksum]`    |
| `06 11` | Set Power Limit                | `03 11 ?? ??` (est.) | `06 11 [float LE] [checksum]`    |
| `06 09` | Power On Output (toggle)       | `03 09 ?? ??` (est.) | `06 09 [00/01] ?? [checksum]`    |
| `06 0C` | Full Charge Auto-Stop (toggle) | `03 0C 01 0D`        | `06 0C [00/01] 00 00 [checksum]` |
| `06 0D` | Two-Stage Switch (toggle)      | `03 0D ?? ??` (est.) | `06 0D [00/01] ?? [checksum]`    |
| `06 0E` | Manual Control (toggle)        | `03 0E ?? ??` (est.) | `06 0E [00/01] ?? [checksum]`    |
| `06 14` | Boolean toggle (confirmed)     | `03 14 01 15`        | `06 14 [00/01] 00 00 [checksum]` |
| `02 06` | Query Status                   | `30 06 [telemetry]`  | `02 06 06`                       |
