#!/usr/bin/env python3
"""
Анализ команды set_max_charging_voltage для Veteran/NOSFET.

Алгоритм:
1. Подключиться к колесу
2. Дождаться Settings пакета (0x08) — прочитать текущий charging_stop_voltage
3. Спросить пользователя какое напряжение выставить
4. Построить команду с разными значениями byte24 (перебор формул)
5. Отправить команду
6. Дождаться следующего Settings — прочитать новое charging_stop_voltage
7. Вычислить обратно: какой offset и коэффициент дают правильный результат

Usage:
    python3 tools/ble_charge_voltage_probe.py          # ищет NF/LK/Veteran
    python3 tools/ble_charge_voltage_probe.py "NF0406"
    python3 tools/ble_charge_voltage_probe.py "Veteran Lynx"
"""

import asyncio
import struct
import sys
import zlib
from bleak import BleakScanner, BleakClient

SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR_UUID    = "0000ffe1-0000-1000-8000-00805f9b34fb"
CHAR_WRITE   = "0000ffe1-0000-1000-8000-00805f9b34fb"

HEADER = bytes([0xDC, 0x5A, 0x5C])
KNOWN_NAMES = ["veteran", "nosfet", "nf", "lk"]

# charge_stop_voltage_offset из протокола (для чтения входящих пакетов)
# Lynx: 682, Aero: -70
DECODE_OFFSETS = {"lynx": 682, "aero": -70, "auto": None}


def crc32_append(payload: bytes) -> bytes:
    crc = zlib.crc32(payload) & 0xFFFFFFFF
    return payload + struct.pack(">I", crc)


def build_charge_packet(byte24: int) -> bytes:
    payload = bytes([
        0x4C, 0x64, 0x41, 0x70, 0x1D, 0x01, 0x02,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
        0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80,
        byte24 & 0xFF,
    ])
    return crc32_append(payload)


def check_crc(p: bytes) -> bool:
    calc = zlib.crc32(p[:-4]) & 0xFFFFFFFF
    recv = struct.unpack(">I", p[-4:])[0]
    return calc == recv


def u16be(d, i):
    return struct.unpack_from(">H", d, i)[0]


def decode_charge_stop(raw_u16: int, offset: float) -> float:
    return (raw_u16 + offset) / 10.0


class VeteranProbe:
    def __init__(self):
        self.buf = bytearray()
        self.settings_queue: asyncio.Queue = asyncio.Queue()

    def on_notify(self, sender, raw: bytearray):
        if len(self.buf) + len(raw) > 512:
            self.buf.clear()
        self.buf.extend(raw)
        pos = 0
        while pos + 3 <= len(self.buf):
            if self.buf[pos:pos+3] != HEADER:
                pos += 1
                continue
            if pos + 4 > len(self.buf):
                break
            pkt_len = 3 + 1 + self.buf[pos + 3]
            if pos + pkt_len > len(self.buf):
                break
            pkt = bytes(self.buf[pos:pos + pkt_len])
            if check_crc(pkt) and len(pkt) > 46:
                subtype = pkt[46]
                if subtype == 0x08 and len(pkt) >= 67:
                    self.settings_queue.put_nowait(pkt)
            pos += pkt_len
        if pos > 0:
            del self.buf[:pos]

    async def wait_settings(self, timeout=8.0) -> bytes | None:
        # Слить накопившиеся старые пакеты
        while not self.settings_queue.empty():
            self.settings_queue.get_nowait()
        try:
            return await asyncio.wait_for(self.settings_queue.get(), timeout)
        except asyncio.TimeoutError:
            return None

    def parse_settings(self, pkt: bytes, decode_offset: float) -> dict:
        raw_u16 = u16be(pkt, 63)
        charge_v = decode_charge_stop(raw_u16, decode_offset)
        return {
            "raw_u16_63": raw_u16,
            "charge_voltage": charge_v,
            "headlight": pkt[47] == 0x01,
            "low_power": pkt[60],
            "high_speed": pkt[61],
            "byte64": pkt[64],
            "byte65": pkt[65],
        }

    def print_settings(self, label: str, info: dict):
        print(f"\n  [{label}]")
        print(f"    charging_stop raw u16[63] = {info['raw_u16_63']} (0x{info['raw_u16_63']:04x})")
        print(f"    charging_stop voltage     = {info['charge_voltage']:.1f} V")
        print(f"    headlight = {info['headlight']},  low_power = {info['low_power']:#04x},  high_speed = {info['high_speed']:#04x}")
        print(f"    byte[64]={info['byte64']}  byte[65]={info['byte65']}")


async def find_device(name_filter: str | None):
    print("Сканирую BLE (6с)...")
    devices = await BleakScanner.discover(timeout=6)
    if name_filter:
        candidates = [d for d in devices if d.name and name_filter.lower() in d.name.lower()]
    else:
        candidates = [d for d in devices if d.name and any(k in d.name.lower() for k in KNOWN_NAMES)]

    if not candidates:
        print("Устройство не найдено. Видимые:")
        for d in sorted(devices, key=lambda x: x.name or ""):
            if d.name:
                print(f"  {d.address}  {d.name}")
        return None

    if len(candidates) == 1:
        return candidates[0]

    print("Найдено несколько:")
    for i, d in enumerate(candidates):
        print(f"  [{i}] {d.name}  ({d.address})")
    idx = int(input("Выбери номер: ").strip())
    return candidates[idx]


async def main():
    name_filter = sys.argv[1] if len(sys.argv) > 1 else None
    target = await find_device(name_filter)
    if not target:
        return

    # Определяем decode_offset по имени устройства
    name_lower = (target.name or "").lower()
    if "lynx" in name_lower or "lk" in name_lower:
        decode_offset = 682.0
        wheel_type = "Lynx"
    else:
        decode_offset = -70.0
        wheel_type = "Aero/NOSFET"

    print(f"\nПодключаюсь к '{target.name}' ({target.address})")
    print(f"Тип колеса: {wheel_type}, decode_offset={decode_offset}")

    probe = VeteranProbe()

    async with BleakClient(target) as client:
        await client.start_notify(CHAR_UUID, probe.on_notify)
        print("Подключено. Жду Settings пакет...")

        pkt = await probe.wait_settings(timeout=10)
        if pkt is None:
            print("Settings пакет не получен")
            return

        before = probe.parse_settings(pkt, decode_offset)
        probe.print_settings("ДО команды", before)
        current_v = before["charge_voltage"]

        # Предлагаем тестовое напряжение (±1V от текущего)
        test_v = current_v - 1.0
        inp = input(f"\nВыставить напряжение [{test_v:.1f}V] (Enter = принять, или введи своё): ").strip()
        if inp:
            test_v = float(inp)

        print(f"\nБудем выставлять {test_v:.1f}V")
        print("Перебираем возможные формулы для byte24:")

        # Перебор формул: byte24 = (voltage - offset) * factor
        candidates = []
        for offset in [145.0, 126.0, 100.0, 110.0, 120.0, decode_offset]:
            for factor in [10.0, 1.0]:
                val = (test_v - offset) * factor
                b = round(val)
                if 0 <= b <= 255:
                    candidates.append((offset, factor, b))
                    print(f"  offset={offset:6.1f}  factor={factor}  byte24={b}  (raw={val:.2f})")

        # Используем текущую формулу Lynx (offset=145, factor=10) как дефолт
        default_offset = 145.0
        default_byte24 = max(0, min(255, round((test_v - default_offset) * 10)))
        print(f"\nДефолтная формула (offset=145, *10): byte24={default_byte24}")

        inp2 = input(f"Введи byte24 для отправки [{default_byte24}]: ").strip()
        byte24 = int(inp2) if inp2 else default_byte24

        cmd = build_charge_packet(byte24)
        print(f"\nОтправляю команду: byte24={byte24}, hex={cmd.hex()}")

        await client.write_gatt_char(CHAR_WRITE, cmd, response=False)
        print("Команда отправлена. Жду подтверждение в Settings пакете...")

        pkt2 = await probe.wait_settings(timeout=10)
        if pkt2 is None:
            print("Ответный Settings пакет не получен")
            return

        after = probe.parse_settings(pkt2, decode_offset)
        probe.print_settings("ПОСЛЕ команды", after)

        # Анализ
        raw_after = after["raw_u16_63"]
        actual_v = after["charge_voltage"]
        print(f"\n{'='*60}")
        print(f"  Отправлен byte24 = {byte24}")
        print(f"  Выставляли:     {test_v:.1f} V")
        print(f"  Получили:       {actual_v:.1f} V  (raw={raw_after})")
        print(f"\n  Обратный расчёт — какой offset даёт правильный результат:")
        print(f"    byte24 = (target - offset) * 10")
        print(f"    => offset = target - byte24/10 = {test_v:.1f} - {byte24/10:.1f} = {test_v - byte24/10:.2f}")
        print(f"\n  Проверка всех offset по фактическому raw_u16={raw_after}:")
        for off in [682.0, -70.0, 145.0, 0.0]:
            v = (raw_after + off) / 10.0
            print(f"    raw + {off:>6.1f} = {raw_after + off:>6}  -> {v:.1f} V")

        await client.stop_notify(CHAR_UUID)

if __name__ == "__main__":
    asyncio.run(main())
