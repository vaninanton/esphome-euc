#!/usr/bin/env python3
"""
Следит за изменениями в Settings пакете (sub-type 0x08).
При каждом изменении любого байта печатает diff и полный байтовый дамп.

Удобно для реверса: меняй настройки на колесе, смотри что меняется.

Usage:
    python3 tools/ble_watch_settings.py "NF0406"
    python3 tools/ble_watch_settings.py "Veteran Lynx"
"""

import asyncio, struct, sys, zlib
from bleak import BleakScanner, BleakClient

CHAR_UUID    = "0000ffe1-0000-1000-8000-00805f9b34fb"
HEADER       = bytes([0xDC, 0x5A, 0x5C])
KNOWN_NAMES  = ["veteran", "nosfet", "nf", "lk"]

# Известные поля в Settings пакете (offset → label)
SETTINGS_LABELS = {
    46: "subtype",
    47: "headlight_front (0=off,1=L1,2=L2,3=L3)",
    50: "pedal_hardness",
    52: "stop_speed",
    53: "pwm_limit",
    55: "screen_backlight",
    60: "low_power_mode",
    61: "high_speed_mode",
    62: "cut_off_angle",
    63: "charge_stop_hi",
    64: "charge_stop_lo",
    65: "charge_voltage_base",
    66: "tho_ra",
    68: "accel_limit",
}

# Известные поля в LIVE пакете (offset → label)
LIVE_LABELS = {
    46: "subtype",
    47: "? (watch for headlight_rear)",
    48: "? (watch)",
    49: "packet_counter",  # монотонно растёт u8
    50: "? (watch)",
    51: "? (watch)",
    52: "? (watch)",
    53: "? (watch)",
    54: "? (watch)",
    55: "? (watch)",
    56: "? (watch)",
    57: "? (watch)",
    58: "? (watch)",
    59: "temp_ctrl_hi",
    60: "temp_ctrl_lo",
    61: "? (watch)",
    62: "? (watch)",
    63: "? (watch)",
    64: "? (watch)",
    65: "? (watch)",
    66: "? (watch)",
    67: "? (watch)",
    68: "? (watch)",
    69: "bms_left_cur_hi",
    70: "bms_left_cur_lo",
    71: "bms_right_cur_hi",
    72: "bms_right_cur_lo",
}

def check_crc(p: bytes) -> bool:
    return (zlib.crc32(p[:-4]) & 0xFFFFFFFF) == struct.unpack(">I", p[-4:])[0]

def u16be(d, i):
    return struct.unpack_from(">H", d, i)[0]

def fmt_byte(offset: int, val: int) -> str:
    label = SETTINGS_LABELS.get(offset, "?")
    return f"[{offset:2d}] {val:#04x} ({val:3d})  {label}"

def print_settings(pkt: bytes, label: str = "", labels: dict = None):
    if labels is None:
        labels = SETTINGS_LABELS
    size = len(pkt)
    print(f"\n{'─'*60}")
    if label:
        print(f"  {label}")
    print(f"  Размер: {size} байт")
    print(f"  Байты [46..{size-5}]:")
    for i in range(46, size - 4):
        val = pkt[i]
        known = labels.get(i, "")
        marker = f"  ← {known}" if known else ""
        print(f"    [{i:2d}] {val:#04x} ({val:3d}){marker}")

def print_diff(prev: bytes, curr: bytes, labels: dict, ignore_range: tuple | None = None):
    changed = []
    for i in range(min(len(prev), len(curr)) - 4):
        if ignore_range and ignore_range[0] <= i < ignore_range[1]:
            continue
        if prev[i] != curr[i]:
            changed.append(i)
    if not changed:
        return
    print(f"\n{'━'*60}")
    print(f"  ИЗМЕНЕНИЕ ({len(changed)} байт):")
    for i in changed:
        label = labels.get(i, "?")
        print(f"    [{i:2d}] {prev[i]:#04x}({prev[i]:3d}) → {curr[i]:#04x}({curr[i]:3d})  {label}")


class Watcher:
    def __init__(self, watch: str = "settings"):
        self.buf = bytearray()
        self.prev: bytes | None = None
        self.count = 0
        # "settings" — следим за 0x08; "live" — следим за 0x00/0x04
        self.watch = watch

    def on_notify(self, sender, raw: bytearray):
        if len(self.buf) + len(raw) > 512:
            self.buf.clear()
        self.buf.extend(raw)
        pos = 0
        while pos + 3 <= len(self.buf):
            if self.buf[pos:pos+3] != HEADER:
                pos += 1; continue
            if pos + 4 > len(self.buf): break
            pkt_len = 3 + 1 + self.buf[pos+3]
            if pos + pkt_len > len(self.buf): break
            pkt = bytes(self.buf[pos:pos+pkt_len])
            if check_crc(pkt) and len(pkt) > 46:
                st = pkt[46]
                if self.watch == "settings" and st == 0x08:
                    self.handle(pkt, SETTINGS_LABELS)
                elif self.watch == "live" and st in (0x00, 0x04):
                    self.handle(pkt, LIVE_LABELS)
            pos += pkt_len
        if pos > 0:
            del self.buf[:pos]

    def handle(self, pkt: bytes, labels: dict):
        self.count += 1
        if self.prev is None:
            print_settings(pkt, f"Начальное состояние (0x{pkt[46]:02x})", labels)
            self.prev = pkt
            return
        # subtype [46] чередуется 0x00/0x04, packet_counter [49] растёт каждый пакет,
        # [57..58] и [62..64] постоянно шумят (быстро меняющиеся величины)
        LIVE_NOISY = {46, 49, 57, 58, 62, 63, 64}
        changed = [
            i for i in range(47, min(len(pkt), len(self.prev)) - 4)
            if pkt[i] != self.prev[i] and i not in LIVE_NOISY
        ] if self.watch == "live" else None

        if self.watch == "live":
            if changed:
                print(f"\n{'━'*60}")
                print(f"  ИЗМЕНЕНИЕ LIVE ({len(changed)} байт):")
                for i in changed:
                    label = labels.get(i, "?")
                    print(f"    [{i:2d}] {self.prev[i]:#04x}({self.prev[i]:3d}) → {pkt[i]:#04x}({pkt[i]:3d})  {label}")
                self.prev = pkt
        else:
            payload_cur = pkt[46:len(pkt)-4]
            payload_prv = self.prev[46:len(self.prev)-4]
            if payload_cur != payload_prv:
                print_diff(self.prev, pkt, labels)
                self.prev = pkt


async def find_device(name_filter):
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
    args = sys.argv[1:]
    watch_mode = "settings"
    name_filter = None
    for a in args:
        if a == "--live":
            watch_mode = "live"
        else:
            name_filter = a

    target = await find_device(name_filter)
    if not target:
        return

    watcher = Watcher(watch=watch_mode)
    mode_desc = "Settings (0x08)" if watch_mode == "settings" else "LIVE (0x00/0x04)"
    print(f"Подключаюсь к '{target.name}'... режим: {mode_desc}")

    async with BleakClient(target) as client:
        notify_char = next(
            c.uuid for s in client.services for c in s.characteristics
            if "notify" in c.properties
        )
        await client.start_notify(notify_char, watcher.on_notify)
        print(f"Слушаю {mode_desc} пакеты. Меняй настройки на колесе. Ctrl+C для выхода.\n")
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(notify_char)
        print(f"\nВсего Settings пакетов получено: {watcher.count}")


if __name__ == "__main__":
    asyncio.run(main())
