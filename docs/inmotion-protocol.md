# Inmotion EUC BLE Protocol

This document describes the Bluetooth Low Energy (BLE) protocol used to communicate with Inmotion electric unicycles (e.g. Inmotion V11). The protocol is reverse‑engineered from device behavior and is intended for integration with ESPHome and similar systems. It uses the **Nordic UART Service (NUS)** for transport.

---

## 1. Transport

| Parameter | Value |
|-----------|--------|
| **Service UUID** | `6E400001-B5A3-F393-E0A9-E50E24DCCA9E` (Nordic UART Service) |
| **Write characteristic** | `6E400002-B5A3-F393-E0A9-E50E24DCCA9E` (TX — host writes, wheel reads) |
| **Notify characteristic** | `6E400003-B5A3-F393-E0A9-E50E24DCCA9E` (RX — wheel writes, host receives) |
| **Role** | Central (phone/ESP) connects to the wheel; the host sends a request command periodically; the wheel responds with notifications. |

Data is exchanged as raw byte arrays. The host **polls** by writing a short command to the TX characteristic; the wheel sends back a **single notification** per request containing telemetry. There is no stream framing in the current implementation — each notification is treated as one complete packet.

---

## 2. Request (Host → Wheel)

To receive telemetry, the host must write the following bytes to the **write characteristic** (`6E400002-...`):

| Bytes (hex) | Description |
|-------------|-------------|
| `AA AA 14 01 04 11` | Request packet (live data / status). |

**Typical usage:** send this request at an interval (e.g. every 1 second). The wheel then sends one notification with the current state.

---

## 3. Response Packet (Wheel → Host)

The wheel replies via the **notify characteristic** (`6E400003-...`). The notification payload is one packet. The current implementation does **not** perform length-based framing or CRC checks; it assumes the notification contains the full packet.

### 3.1 Packet validation

Before parsing, the packet is accepted only if:

- `bytes[2] == 0x14`
- `(bytes[4] & 0x7F) == 0x04`

So the first bytes act as a type/subtype identifier. The packet must have at least **62 bytes** (indices 0–61 are used). Invalid packets are ignored.

### 3.2 Byte order

- **16‑bit values** in this protocol use **big‑endian** (high byte first), e.g.  
  `value = (bytes[offset] << 8) | bytes[offset + 1]`.
- **Signed 16‑bit (current):** same byte order, interpreted as `int16_t`; sign extension may apply when shifting (see implementation).

---

## 4. Response Payload Layout

All offsets are **0‑based** from the start of the notification payload.

### 4.1 Identifiers (no scaling)

| Offset | Size | Type | Description |
|--------|------|------|-------------|
| 2 | 1 | byte | Packet type; must be `0x14` for this response. |
| 4 | 1 | byte | Subtype / flags; low 7 bits must be `0x04` for the parsed format. |

### 4.2 Telemetry fields

| Offset | Size | Type | Scale | Description |
|--------|------|------|--------|-------------|
| 5–6 | 2 | uint16 BE | ÷ 100 → V | Battery voltage (e.g. 8440 → 84.40 V). |
| 7–8 | 2 | int16 BE | ÷ 100 → A | Current (signed); positive = discharge, negative = charge. |
| 33–34 | 2 | uint16 BE | ÷ 100 → % | Battery percentage (e.g. 8500 → 85.00%). |
| 61 | 1 | byte | bits | Status flags: bit 7 = charging, bit 6 = lifted. |

**Formulas (from code):**

- Voltage (V): `((bytes[6] & 0xFF) << 8) | (bytes[5] & 0xFF)` then divide by 100.
- Current (A): `(bytes[8] << 8) | (bytes[7] & 0xFF)` as int16, then divide by 100.
- Battery %: `((bytes[34] & 0xFF) << 8) | (bytes[33] & 0xFF)` then divide by 100.
- Power (W): `(current_A) * (voltage_V)` (current and voltage already in physical units after scale).
- Charging: `(bytes[61] >> 7) & 0x01` → 1 = charging, 0 = not charging.
- Lifted: `(bytes[61] >> 6) & 0x01` → 1 = wheel lifted, 0 = on ground.

---

## 5. Summary Table (Response)

| Offset | Size | Type | Scale / meaning | Description |
|--------|------|------|------------------|-------------|
| 2 | 1 | byte | 0x14 | Packet type. |
| 4 | 1 | byte | 0x04 (low 7 bits) | Subtype. |
| 5 | 2 | uint16 BE | ÷ 100 | Voltage (V). |
| 7 | 2 | int16 BE | ÷ 100 | Current (A). |
| 33 | 2 | uint16 BE | ÷ 100 | Battery percentage. |
| 61 | 1 | byte | bits 7–6 | Charging (7), Lifted (6). |

---

## 6. Implementation Notes

- **Polling:** Send the request `AA AA 14 01 04 11` at a fixed interval (e.g. 1000 ms). Do not rely on unsolicited notifications only.
- **Packet length:** Ensure the notification buffer has at least 62 bytes before reading offsets 33, 61, etc., to avoid out-of-range access.
- **Validation:** Ignore notifications where `bytes[2] != 0x14` or `(bytes[4] & 0x7F) != 0x04`.
- **No CRC:** The current implementation does not verify checksum or CRC; trust only validated packet type/subtype.
- **Models:** This layout is derived from the Inmotion V11 integration; other models (e.g. V12) may use the same or different type/subtype and offsets — validate with captures if needed.

This document reflects the behavior implemented in the ESPHome Inmotion parser (`parser_inmotionv2.h`); protocol details may vary by wheel model and firmware version.
