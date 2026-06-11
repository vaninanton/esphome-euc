#!/usr/bin/env python3
"""
Анализ нераспаршенных байт в BLE-пакетах Veteran/NOSFET.

Для каждого sub-type показывает:
- какие байты уже известны (с названием поля)
- какие байты не распознаны (со значениями из нескольких пакетов)
- если байт стабилен — выделяет его как кандидата на константу
- если байт меняется — показывает диапазон значений

Usage:
    python3 tools/ble_unknown_bytes.py              # ищет NF/LK/Veteran
    python3 tools/ble_unknown_bytes.py "NF0406"
    python3 tools/ble_unknown_bytes.py "Veteran Lynx"
    python3 tools/ble_unknown_bytes.py "NF0406" --seconds 30
"""

import asyncio
import struct
import sys
import zlib
from collections import defaultdict

CHAR_UUID = "0000ffe1-0000-1000-8000-00805f9b34fb"
HEADER = bytes([0xDC, 0x5A, 0x5C])
KNOWN_NAMES = ["veteran", "nosfet", "nf", "lk"]

# ── Известные поля по абсолютному offset в пакете ──────────────────────────
# Формат: offset -> (name, size_bytes)
# Для u16/u32 помечаем все занятые байты.

KNOWN_FIELDS_COMMON = {
    0: ("header[0]", 1), 1: ("header[1]", 1), 2: ("header[2]", 1),
    3: ("length", 1),
    4: ("voltage_hi", 1), 5: ("voltage_lo", 1),
    6: ("speed_hi", 1), 7: ("speed_lo", 1),
    8: ("mileage_cur[2]", 1), 9: ("mileage_cur[3]", 1),
    10: ("mileage_cur[0]", 1), 11: ("mileage_cur[1]", 1),
    12: ("mileage_tot[2]", 1), 13: ("mileage_tot[3]", 1),
    14: ("mileage_tot[0]", 1), 15: ("mileage_tot[1]", 1),
    16: ("phase_current_hi", 1), 17: ("phase_current_lo", 1),
    18: ("temp_motor_hi", 1), 19: ("temp_motor_lo", 1),
    20: ("auto_off_hi", 1), 21: ("auto_off_lo", 1),
    22: ("charging_hi", 1), 23: ("charging_lo", 1),
    24: ("speed_alert_hi", 1), 25: ("speed_alert_lo", 1),
    26: ("speed_tiltback_hi", 1), 27: ("speed_tiltback_lo", 1),
    28: ("fw_hi", 1), 29: ("fw_lo", 1),
    30: ("pedals_mode_hi", 1), 31: ("pedals_mode_lo", 1),
    32: ("pitch_hi", 1), 33: ("pitch_lo", 1),
    34: ("pwm_hi", 1), 35: ("pwm_lo", 1),
    # 36..45 — неизвестно
    46: ("subtype", 1),
}

# Дополнительные известные поля по sub-type
KNOWN_FIELDS_BY_SUBTYPE = {
    # LIVE 0x00 / 0x04
    0x00: {
        59: ("temp_ctrl_hi", 1), 60: ("temp_ctrl_lo", 1),
        69: ("bms_left_cur_hi", 1), 70: ("bms_left_cur_lo", 1),
        71: ("bms_right_cur_hi", 1), 72: ("bms_right_cur_lo", 1),
    },
    # BMS LEFT CELLS 1-15
    0x01: {i: (f"bms_L_cell{(i-53)//2+1}_{'hi' if (i-53)%2==0 else 'lo'}", 1)
           for i in range(53, 83)},
    # BMS LEFT CELLS 16-30
    0x02: {i: (f"bms_L_cell{(i-53)//2+16}_{'hi' if (i-53)%2==0 else 'lo'}", 1)
           for i in range(53, 83)},
    # BMS LEFT TEMPS + CELLS 31-36
    0x03: {
        **{i: (f"bms_L_temp{(i-47)//2+1}_{'hi' if (i-47)%2==0 else 'lo'}", 1)
           for i in range(47, 59)},
        **{i: (f"bms_L_cell{(i-59)//2+31}_{'hi' if (i-59)%2==0 else 'lo'}", 1)
           for i in range(59, 71)},
        **{i: (f"bms_L_cell{(i-71)//2+37}_{'hi' if (i-71)%2==0 else 'lo'}", 1)
           for i in range(71, 83)},
    },
    # BMS RIGHT CELLS 1-15
    0x05: {i: (f"bms_R_cell{(i-53)//2+1}_{'hi' if (i-53)%2==0 else 'lo'}", 1)
           for i in range(53, 83)},
    # BMS RIGHT CELLS 16-30
    0x06: {i: (f"bms_R_cell{(i-53)//2+16}_{'hi' if (i-53)%2==0 else 'lo'}", 1)
           for i in range(53, 83)},
    # BMS RIGHT TEMPS + CELLS 31-36
    0x07: {
        **{i: (f"bms_R_temp{(i-47)//2+1}_{'hi' if (i-47)%2==0 else 'lo'}", 1)
           for i in range(47, 59)},
        **{i: (f"bms_R_cell{(i-59)//2+31}_{'hi' if (i-59)%2==0 else 'lo'}", 1)
           for i in range(59, 71)},
        **{i: (f"bms_R_cell{(i-71)//2+37}_{'hi' if (i-71)%2==0 else 'lo'}", 1)
           for i in range(71, 83)},
    },
    # SETTINGS 0x08
    0x08: {
        47: ("headlight", 1),
        60: ("low_power_mode", 1),
        61: ("high_speed_mode", 1),
        62: ("cut_off_angle", 1),
        63: ("charge_stop_v_hi", 1), 64: ("charge_stop_v_lo", 1),
        66: ("tho_ra", 1),
    },
}
# LIVE ALT — те же поля что 0x00
KNOWN_FIELDS_BY_SUBTYPE[0x04] = KNOWN_FIELDS_BY_SUBTYPE[0x00]

SUBTYPE_NAMES = {
    0x00: "LIVE", 0x04: "LIVE_ALT",
    0x01: "BMS_L_CELLS_1-15", 0x02: "BMS_L_CELLS_16-30", 0x03: "BMS_L_TEMPS+31-36",
    0x05: "BMS_R_CELLS_1-15", 0x06: "BMS_R_CELLS_16-30", 0x07: "BMS_R_TEMPS+31-36",
    0x08: "SETTINGS",
}

# ── CRC / reassembly ────────────────────────────────────────────────────────

def check_crc(p: bytes) -> bool:
    return (zlib.crc32(p[:-4]) & 0xFFFFFFFF) == struct.unpack(">I", p[-4:])[0]


# ── Анализ ──────────────────────────────────────────────────────────────────

def analyze(packets_by_subtype: dict[int, list[bytes]]):
    for subtype in sorted(packets_by_subtype):
        pkts = packets_by_subtype[subtype]
        name = SUBTYPE_NAMES.get(subtype, f"UNKNOWN_0x{subtype:02x}")
        size = len(pkts[0])
        print(f"\n{'═'*72}")
        print(f"  sub-type 0x{subtype:02x}  {name}  размер={size}  пакетов={len(pkts)}")
        print(f"{'═'*72}")

        known = {**KNOWN_FIELDS_COMMON, **KNOWN_FIELDS_BY_SUBTYPE.get(subtype, {})}

        # Собираем значения каждого байта по всем пакетам
        byte_values: dict[int, set[int]] = defaultdict(set)
        for pkt in pkts:
            for i, b in enumerate(pkt[:-4]):  # без CRC
                byte_values[i].add(b)

        # Группируем неизвестные байты в диапазоны
        unknown_ranges: list[tuple[int, int]] = []
        in_range = False
        for i in range(size - 4):
            if i not in known:
                if not in_range:
                    range_start = i
                    in_range = True
            else:
                if in_range:
                    unknown_ranges.append((range_start, i - 1))
                    in_range = False
        if in_range:
            unknown_ranges.append((range_start, size - 5))

        if not unknown_ranges:
            print("  Все байты распознаны ✓")
            continue

        print(f"  Неизвестные байты ({sum(b-a+1 for a,b in unknown_ranges)} шт):\n")
        print(f"  {'offset':>6}  {'hex':>4}  {'dec':>3}  {'стабил':>6}  поле/значения")
        print(f"  {'─'*64}")

        for r_start, r_end in unknown_ranges:
            # Заголовок диапазона
            if r_end > r_start:
                print(f"  [{r_start}..{r_end}]")
            for i in range(r_start, r_end + 1):
                vals = byte_values[i]
                stable = len(vals) == 1
                val_repr = f"0x{next(iter(vals)):02x}" if stable else f"{sorted(vals)}"
                flag = "CONST" if stable else "varies"

                # Подсказки по значению
                hints = []
                if stable:
                    v = next(iter(vals))
                    if v == 0x80:
                        hints.append("0x80=not_supported?")
                    elif v == 0x00:
                        hints.append("zero/false")
                    elif v == 0xFF:
                        hints.append("0xFF=disabled?")
                else:
                    vlist = sorted(vals)
                    vmin, vmax = vlist[0], vlist[-1]
                    span = vmax - vmin
                    # Похоже на температуру (0..120 в сотых → 0..12000, или прямо °C*100)
                    if 2 <= len(vals) <= 5 and 0 <= vmin and vmax <= 120:
                        hints.append(f"range {vmin}..{vmax}, maybe °C or %?")
                    elif span > 0:
                        hints.append(f"range {vmin}..{vmax}")
                    # Проверяем пары байт как u16 BE
                    if i + 1 <= r_end:
                        u16_vals = set()
                        for pkt in pkts:
                            u16_vals.add(struct.unpack_from(">H", pkt, i)[0])
                        u16_min, u16_max = min(u16_vals), max(u16_vals)
                        if len(u16_vals) > 1:
                            hints.append(f"u16be: {u16_min}..{u16_max} ({u16_min/100:.1f}..{u16_max/100:.1f} if ÷100)")

                hint_str = "  ← " + ", ".join(hints) if hints else ""
                print(f"  [{i:>4}]  {val_repr:>4}  {('?' if not stable else next(iter(vals))):>3}  {flag:>6}  {hint_str}")

        # Показываем hex первого пакета для ручного анализа
        pkt = pkts[0]
        hex_parts = []
        for i, b in enumerate(pkt[:-4]):
            if i in known:
                hex_parts.append(f"\033[2m{b:02x}\033[0m")  # dim = известный
            else:
                hex_parts.append(f"\033[1;33m{b:02x}\033[0m")  # yellow = неизвестный
        crc_hex = " ".join(f"{b:02x}" for b in pkt[-4:])
        print(f"\n  Hex (жёлтый = неизвестный):")
        # Без ANSI если не tty
        if sys.stdout.isatty():
            print(f"  {' '.join(hex_parts)} \033[2m{crc_hex}\033[0m [CRC]")
        else:
            print(f"  {pkt[:-4].hex(' ')}  {crc_hex} [CRC]")


# ── BLE ─────────────────────────────────────────────────────────────────────

class Collector:
    def __init__(self):
        self.buf = bytearray()
        self.packets: dict[int, list[bytes]] = defaultdict(list)
        self.total = 0

    def on_notify(self, sender, raw: bytearray):
        if len(self.buf) + len(raw) > 512:
            self.buf.clear()
        self.buf.extend(raw)
        pos = 0
        while pos + 3 <= len(self.buf):
            if self.buf[pos:pos+3] != HEADER:
                pos += 1; continue
            if pos + 4 > len(self.buf): break
            pkt_len = 3 + 1 + self.buf[pos + 3]
            if pos + pkt_len > len(self.buf): break
            pkt = bytes(self.buf[pos:pos + pkt_len])
            if check_crc(pkt) and len(pkt) > 46:
                subtype = pkt[46]
                self.packets[subtype].append(pkt)
                self.total += 1
            pos += pkt_len
        if pos > 0:
            del self.buf[:pos]


async def find_device(name_filter):
    from bleak import BleakScanner
    print("Сканирую BLE (6с)...")
    devices = await BleakScanner.discover(timeout=6)
    if name_filter:
        candidates = [d for d in devices if d.name and name_filter.lower() in d.name.lower()]
    else:
        candidates = [d for d in devices if d.name and any(k in d.name.lower() for k in KNOWN_NAMES)]
    if not candidates:
        print("Устройство не найдено. Видимые:")
        for d in sorted(devices, key=lambda x: x.name or ""):
            if d.name: print(f"  {d.address}  {d.name}")
        return None
    if len(candidates) == 1:
        return candidates[0]
    print("Найдено несколько:")
    for i, d in enumerate(candidates):
        print(f"  [{i}] {d.name}  ({d.address})")
    return candidates[int(input("Выбери номер: ").strip())]


async def main():
    from bleak import BleakClient

    name_filter = None
    seconds = 20
    args = sys.argv[1:]
    while args:
        a = args.pop(0)
        if a == "--seconds" and args:
            seconds = int(args.pop(0))
        else:
            name_filter = a

    target = await find_device(name_filter)
    if not target:
        return

    collector = Collector()
    print(f"Подключаюсь к '{target.name}'...")

    async with BleakClient(target) as client:
        # Найти характеристику с notify (не хардкодим UUID — Lynx может быть занят ESP32)
        notify_char = None
        for s in client.services:
            for c in s.characteristics:
                if "notify" in c.properties:
                    notify_char = c.uuid
                    break
            if notify_char:
                break
        if not notify_char:
            print("Не найдена характеристика с notify")
            return
        print(f"Notify характеристика: {notify_char}")
        await client.start_notify(notify_char, collector.on_notify)
        print(f"Собираю пакеты {seconds}с...  (Ctrl+C для досрочной остановки)")
        try:
            await asyncio.sleep(seconds)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(notify_char)

    print(f"\nСобрано {collector.total} пакетов, "
          f"{len(collector.packets)} sub-types: "
          f"{[hex(s) for s in sorted(collector.packets)]}")

    analyze(collector.packets)


if __name__ == "__main__":
    asyncio.run(main())
