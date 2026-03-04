# Veteran EUC BLE Protocol

This document describes the Bluetooth Low Energy (BLE) protocol used to communicate with Veteran electric unicycles (e.g. Veteran Lynx, Veteran Sherman L). The protocol is reverse‑engineered from device behavior and is intended for integration with ESPHome and similar systems.

---

## 1. Transport

| Parameter               | Value                                                                                                   |
| ----------------------- | ------------------------------------------------------------------------------------------------------- |
| **Service UUID**        | `FFE0`                                                                                                  |
| **Characteristic UUID** | `FFE1` (notify + write)                                                                                 |
| **Role**                | Central (phone/ESP) connects to the wheel; the wheel sends notifications; the app sends write commands. |

Data is exchanged as raw byte arrays. Incoming data (notifications) must be parsed as framed packets; outgoing data are command packets written to the characteristic.

---

## 2. Incoming Packet Framing

Notifications from the wheel are a stream of bytes. Packets are extracted as follows.

### 2.1 Packet structure

| Field   | Offset | Size | Description                                                               |
| ------- | ------ | ---- | ------------------------------------------------------------------------- |
| Header  | 0      | 3    | Fixed `0xDC 0x5A 0x5C`                                                    |
| Length  | 3      | 1    | Payload length (number of data bytes following this byte, excluding CRC). |
| Payload | 4      | N    | Data; N = value of Length byte.                                           |
| CRC     | 4 + N  | 4    | CRC32 (see below).                                                        |

**Total packet length** = 3 + 1 + N + 4 = **8 + N** bytes, where N = payload length (if Length = payload only).

_Реальная прошивка Veteran Lynx: байт Length = число байт после него (payload + CRC), т.е. payload = Length − 4, итоговая длина пакета = **4 + Length**._

### 2.2 Parsing algorithm

1. Search for the 3‑byte header `0xDC 0x5A 0x5C` in the stream.
2. Read the length byte at offset 3: `length = buffer[3]`.
3. Wait until at least `8 + length` bytes are available (or whatever length your CRC layout requires).
4. Extract one packet, verify CRC, then parse the payload (see below).
5. Remove the consumed bytes from the buffer and repeat.
6. If the buffer grows beyond a safe limit (e.g. 1024 bytes) without a valid header, clear it to avoid stalling.

### 2.3 CRC

- **Algorithm:** CRC‑32 (little‑endian), e.g. `esp_crc32_le()`.
- **Scope:** CRC is computed over the packet **excluding** the CRC field (typically bytes `0` to `packet_length - 4`).
- **Storage:** The 4‑byte CRC is stored at the end of the packet in **big‑endian** order and compared to the computed value. Invalid CRC → discard the packet.

---

## 3. Byte Order Conventions

- **Big‑endian 16‑bit:** high byte first.  
  `value = (bytes[offset] << 8) | bytes[offset + 1]`
- **Big‑endian 32‑bit:** high byte first.  
  `value = (bytes[offset]<<24) | (bytes[offset+1]<<16) | (bytes[offset+2]<<8) | bytes[offset+3]`
- **Mid‑little 32‑bit:** bytes at `offset+2, offset+3, offset, offset+1` form a 32‑bit value in big‑endian order.  
  Used for mileage fields.

---

## 4. Common Payload (Offsets 4–35)

These fields appear in every packet (after the 3‑byte header and 1‑byte length). All offsets below are **from the start of the full packet** (index 0 = first byte of header).

| Offset | Size | Type          | Scale / notes | Description                                                      |
| ------ | ---- | ------------- | ------------- | ---------------------------------------------------------------- |
| 4      | 2    | uint16 BE     | ÷ 100 → V     | Battery voltage (e.g. 15030 → 150.30 V).                         |
| 6      | 2    | int16 BE      | —             | Speed (signed).                                                  |
| 8      | 4    | uint32 mid‑LE | ÷ 1000 → km   | Current trip mileage.                                            |
| 12     | 4    | uint32 mid‑LE | ÷ 1000 → km   | Total mileage.                                                   |
| 16     | 2    | int16 BE      | —             | Phase current.                                                   |
| 18     | 2    | uint16 BE     | ÷ 100 → °C    | Motor temperature.                                               |
| 20     | 2    | uint16 BE     | —             | Auto‑off timer (seconds); 900 = disabled.                        |
| 23     | 1    | bool          | 0x01 = true   | Charging state.                                                  |
| 24     | 2    | uint16 BE     | —             | Speed alert threshold.                                           |
| 26     | 2    | uint16 BE     | —             | Tiltback speed.                                                  |
| 28     | 2    | uint16 BE     | —             | Firmware version code: model = fw/1000, then version components. |
| 30     | 2    | uint16 BE     | value − 100   | Pedals mode (0 = soft, 100 = hard).                              |
| 32     | 2    | int16 BE      | —             | Pitch angle (e.g. ~6248 when on stock pedals).                   |
| 34     | 2    | uint16 BE     | —             | PWM.                                                             |

**Firmware version string** from the 16‑bit value at offset 28:

- `modelVersion = fw / 1000`
- Version string: `"XXX.Y.ZZ"` with `(fw/100)%10` and `fw%100` (exact format may vary).

**Supported models (by modelVersion):**

- **5** – Veteran Lynx
- **6** – Veteran Sherman L

Extended BMS and settings data is only parsed when `modelVersion >= 5`.

---

## 5. Extended Payload (modelVersion ≥ 5)

For models 5 and 6, the payload is longer and the **sub‑type** is at **offset 46** (`bytes[46]`). Only the documented sub‑types are parsed below.

### 5.1 Sub‑type 0x00 or 0x04 — Live data

| Offset | Size | Type      | Scale    | Description                                 |
| ------ | ---- | --------- | -------- | ------------------------------------------- |
| 59     | 2    | uint16 BE | ÷ 100    | Controller temperature (°C).                |
| 69     | 2    | int16 BE  | ÷ (−100) | Left pack current (A); negative = charging. |
| 71     | 2    | int16 BE  | ÷ (−100) | Right pack current (A).                     |

Power (W) can be computed as:  
`voltage × (left_current + right_current) / 100`.

### 5.2 Sub‑type 0x01 — BMS left pack, cells 1–15

| Offset   | Size | Type      | Scale      | Description                           |
| -------- | ---- | --------- | ---------- | ------------------------------------- |
| 53 + 2×k | 2    | uint16 BE | ÷ 1000 → V | Cell voltage, k = 0..14 (cells 1–15). |

### 5.3 Sub‑type 0x02 — BMS left pack, cells 16–30

Same layout as 0x01, but for left pack cells 16–30 (offsets 53–81, step 2).

### 5.4 Sub‑type 0x03 — BMS left pack, temperatures and cells 31–36

| Offset | Size | Type      | Scale  | Description          |
| ------ | ---- | --------- | ------ | -------------------- |
| 47–57  | 6×2  | uint16 BE | ÷ 100  | Temperatures 1–6.    |
| 59–69  | 6×2  | uint16 BE | ÷ 1000 | Cell voltages 31–36. |

### 5.5 Sub‑type 0x05 — BMS right pack, cells 1–15

Same as 0x01 for the right pack (offsets 53–81).

### 5.6 Sub‑type 0x06 — BMS right pack, cells 16–30

Same as 0x02 for the right pack.

### 5.7 Sub‑type 0x07 — BMS right pack, temperatures and cells 31–36

Same as 0x03 for the right pack (temperatures and cells 31–36).

### 5.8 Sub‑type 0x08 — Settings / status

| Offset | Size | Type      | Description                                                               |
| ------ | ---- | --------- | ------------------------------------------------------------------------- |
| 47     | 1    | bool      | Headlight on (0x01 = on).                                                 |
| 51     | 2    | uint16 BE | Gyro level (optional).                                                    |
| 55     | 2    | uint16 BE | Brightness (optional).                                                    |
| 60     | 1    | bool      | Low power mode.                                                           |
| 61     | 1    | bool      | High speed mode.                                                          |
| 62     | 1    | byte      | Cut‑off angle.                                                            |
| 63     | 2    | uint16 BE | Charging stop voltage (raw) + 682 → actual value; divide by 10 for volts. |
| 66     | 1    | byte      | Tho_ra (protocol‑specific).                                               |

---

## 6. Battery Percentage

Two calculation modes are used in the code:

**Linear (optional):**

- Voltage &gt; 148.04 V → 100%
- Voltage &gt; 119.03 V → `(voltage − 119.02) / 29.03`
- Otherwise → 0%

**Default (non‑linear):**

- Voltage &gt; 150.30 V → 100%
- Voltage &gt; 122.40 V → `(voltage − 119.70) / 30.6`
- Voltage &gt; 115.20 V → `(voltage − 115.20) / 81.0`
- Otherwise → 0%

Voltage here is in the raw unit (e.g. 15030 = 150.30 V). Exact thresholds may be model‑specific.

---

## 7. Outgoing Commands (Writes to FFE1)

Commands are sent by writing byte arrays to the same characteristic (FFE1). Format is command‑specific; known examples use a leading signature and trailing CRC.

### 7.1 Headlight on

Raw payload (hex):

`4C 6B 41 70 0D 01 80 80 01 57 ED 3B D5 4C 64 41 70 0D 01 00 80 01 6F F8 32 F9`

### 7.2 Headlight off

Raw payload (hex):

`4C 6B 41 70 0D 01 80 80 00 20 EA 0B 43 4C 64 41 70 0D 01 00 80 00 18 FF 02 6F`

### 7.3 Charge to 151.2 V (max charge) — ON

Raw payload (hex):

`4C 64 41 70 1D 01 02 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 3E 83 26 35 D8`

### 7.4 Charge to 151.2 V — OFF

Raw payload (hex):

`4C 64 41 70 1D 01 02 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 14 58 9D FC 0E`

### 7.5 Max charging voltage (variable)

A template for a configurable max charging voltage (e.g. 147.0–151.2 V) uses a packet that starts with:

`4C 64 41 70 1D 01 02 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80 80`

The voltage is encoded in a later byte (e.g. index 24) and the last 4 bytes are CRC. Exact encoding (e.g. `voltage × 10` and offset) and CRC calculation must match the wheel firmware; the current implementation’s formula is still under review (see code comments around `send_packet`).

---

## 8. Implementation Notes

- **Buffer handling:** Incoming BLE chunks may split or merge packets; always use a sliding buffer and frame by header + length.
- **CRC:** Validate every packet before parsing; ignore or log invalid ones.
- **Model version:** Use `modelVersion` (from offset 28) to decide whether to parse extended and BMS blocks (only for ≥ 5).
- **Sub‑type:** For extended payloads, branch on `bytes[46]` to choose the correct layout (live data, BMS left/right, settings).
- **Optional sensors:** Not all wheels expose all BMS cells or settings; handle missing or null entities to avoid crashes.

This document reflects the behavior implemented in the ESPHome Veteran component; protocol details may vary by wheel model and firmware version.
