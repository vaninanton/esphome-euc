#!/usr/bin/env python3
"""
Дамп BLE-пакетов Veteran/NOSFET.
Подключается к колесу, собирает notify с FFE1, reassembly по заголовку DC5A5C,
выводит hex + разбор sub-type для каждого пакета.

На macOS BLE отдаёт UUID вместо MAC — поиск идёт по имени устройства.

Usage:
    python3 tools/ble_dump_veteran.py              # сканирует и предлагает выбор
    python3 tools/ble_dump_veteran.py "Veteran"    # подключается к первому с "Veteran" в имени
    python3 tools/ble_dump_veteran.py --scan       # только сканирование, без подключения
"""

import asyncio
import sys
import struct
import zlib

# Имена, по которым ищем колесо (подстрока, case-insensitive)
KNOWN_NAMES = ["veteran", "nosfet", "leaperkim", "nf", "lk"]

SERVICE_UUID = "0000ffe0-0000-1000-8000-00805f9b34fb"
CHAR_UUID    = "0000ffe1-0000-1000-8000-00805f9b34fb"

HEADER = bytes([0xDC, 0x5A, 0x5C])

# --- CRC32 (ESP CRC32 LE = standard zlib crc32) ---
def crc32_le(data: bytes) -> int:
    return zlib.crc32(data) & 0xFFFFFFFF

def check_crc(packet: bytes) -> bool:
    if len(packet) < 4:
        return False
    calc = crc32_le(packet[:-4])
    recv = struct.unpack(">I", packet[-4:])[0]
    return calc == recv

# --- Разбор ---
SUBTYPES = {
    0x00: "LIVE",
    0x04: "LIVE_ALT",
    0x01: "BMS_L_CELLS_1-15",
    0x02: "BMS_L_CELLS_16-30",
    0x03: "BMS_L_TEMPS+CELLS_31-36",
    0x05: "BMS_R_CELLS_1-15",
    0x06: "BMS_R_CELLS_16-30",
    0x07: "BMS_R_TEMPS+CELLS_31-36",
    0x08: "SETTINGS",
}

def u16be(data, offset):
    return struct.unpack_from(">H", data, offset)[0]

def s16be(data, offset):
    return struct.unpack_from(">h", data, offset)[0]

def parse_common(data):
    p = 4  # COMMON_PAYLOAD_OFFSET
    voltage     = u16be(data, p + 0) / 100.0
    # mileage: u32 mid-LE (word-swap): bytes [p+6][p+7][p+4][p+5]
    raw_mc = (data[p+6] << 24) | (data[p+7] << 16) | (data[p+4] << 8) | data[p+5]
    raw_mt = (data[p+10] << 24) | (data[p+11] << 16) | (data[p+8] << 8) | data[p+9]
    mileage_cur = raw_mc / 1000.0
    mileage_tot = raw_mt / 1000.0
    temp_motor  = u16be(data, p + 14) / 100.0
    auto_off    = u16be(data, p + 16)
    charging_u16 = u16be(data, p + 18)   # стандарт: u16 BE с p+18 (абс. 22)
    charging_b23 = data[p + 19]           # наш текущий вариант: байт p+19 (абс. 23)
    fw_raw      = u16be(data, p + 24)
    model_ver   = fw_raw // 1000
    fw_str      = f"{model_ver}.{(fw_raw // 100) % 10}.{fw_raw % 100:02d}"

    print(f"  [COMMON] voltage={voltage:.2f}V  mileage={mileage_cur:.3f}/{mileage_tot:.3f}km")
    print(f"           temp_motor={temp_motor:.2f}°C  auto_off={auto_off}s")
    print(f"           charging u16@22={charging_u16} (!=0?{charging_u16!=0})  byte@23={charging_b23} (==1?{charging_b23==1})")
    print(f"           fw={fw_str}  modelVersion={model_ver}")
    return model_ver

def parse_settings(data):
    """Детальный разбор Settings (0x08) — сравниваем несколько вариантов чтения."""
    size = len(data)
    print(f"  [SETTINGS] размер пакета = {size} байт")

    def show(label, offset, value_fmt="u8"):
        if offset >= size:
            print(f"    {label:40s} offset={offset}  -> OUT OF RANGE")
            return
        if value_fmt == "u8":
            val = data[offset]
            print(f"    {label:40s} offset={offset}  raw={val:#04x} ({val})")
        elif value_fmt == "u16be":
            if offset + 1 >= size:
                print(f"    {label:40s} offset={offset}  -> OUT OF RANGE (u16)")
                return
            val = u16be(data, offset)
            print(f"    {label:40s} offset={offset}  raw={val:#06x} ({val})  /10={val/10:.1f}V")
        elif value_fmt == "s8":
            val = struct.unpack_from("b", data, offset)[0]
            print(f"    {label:40s} offset={offset}  raw={val}")

    # Наши константы
    print("  -- наши константы (veteran.h) --")
    show("SETTINGS_HEADLIGHT_OFFSET=47     headlight", 47)
    show("SETTINGS_LOW_POWER_OFFSET=60     low_power", 60)
    show("SETTINGS_HIGH_SPEED_OFFSET=61    high_speed", 61)
    show("SETTINGS_CUT_OFF_OFFSET=62       cut_off", 62)
    show("SETTINGS_CHARGE_V_OFFSET=63      charge_v (u16be)", 63, "u16be")
    show("SETTINGS_THO_RA_OFFSET=66        tho_ra", 66)

    # FreeWheel layout
    print("  -- FreeWheel layout (pnum=8) --")
    show("50  pedalHardness", 50)
    show("52  stopSpeed", 52)
    show("53  pwmLimit", 53)
    show("55  screenBacklight", 55)
    show("57  transportMode", 57)
    show("58  displayUnit (0=km,1=mi)", 58)
    show("59  voltageCorrection (s8)", 59, "s8")
    show("60  lowVoltageMode", 60)
    show("61  highSpeedMode", 61)
    show("63  keyTone", 63)
    show("64  maxChargeVol u8 (0-120 raw)", 64)
    show("65  chargeVoltageBase u8 (0->145V base)", 65)
    show("66  dynamicAssist", 66)
    show("68  accelerationLimit", 68)
    show("69  brakePressureAlarm", 69)

    # Вычислим charging_stop_voltage по двум версиям
    print("  -- Вычисление charging_stop_voltage --")
    if 64 < size:
        max_charge_vol = data[64]
        charge_base    = data[65] if 65 < size else 0
        freewheel_v    = (max_charge_vol + charge_base) / 10.0
        print(f"    FreeWheel: maxChargeVol[64]={max_charge_vol} + chargeVoltageBase[65]={charge_base} = {max_charge_vol+charge_base} -> {freewheel_v:.1f}V")
    if 64 < size:
        our_raw = u16be(data, 63) if 64 < size else 0
        for offset_name, offset, offset_val in [("Lynx +682", 63, 682), ("Aero -70", 63, -70)]:
            if offset + 1 < size:
                raw = u16be(data, offset)
                result = (raw + offset_val) / 10.0
                print(f"    Наш u16be[{offset}]={raw} {offset_name:12s} -> {result:.1f}V")

def parse_live(data):
    size = len(data)
    if size < 73:
        print(f"  [LIVE] слишком короткий: {size} байт (нужно ≥73)")
        return
    temp_ctrl = u16be(data, 59) / 100.0
    bms_l_cur = s16be(data, 69) / -100.0
    bms_r_cur = s16be(data, 71) / -100.0
    byte70    = data[70]
    headlight_map = {0: "Level?/Off", 1: "Level 1", 3: "Level 2", 5: "Level 3", 128: "Off"}
    hl_text = headlight_map.get(byte70, f"unknown({byte70})")
    print(f"  [LIVE]  temp_ctrl={temp_ctrl:.2f}°C")
    print(f"          bms_left_cur={bms_l_cur:.3f}A  bms_right_cur={bms_r_cur:.3f}A")
    print(f"          byte[70]={byte70:#04x} ({byte70}) -> headlight_level={hl_text}")
    print(f"          byte[69..70] as s16be={s16be(data,69)} (это bms_left_current raw)")

def parse_packet(data: bytes):
    size = len(data)
    crc_ok = check_crc(data)
    print(f"\n{'='*70}")
    print(f"Пакет {size} байт  CRC={'OK' if crc_ok else 'FAIL'}")
    print(f"  hex: {data.hex(' ')}")

    if size < 36:
        print("  слишком короткий")
        return

    model_ver = parse_common(data)

    if model_ver >= 5 and size >= 47:
        subtype = data[46]
        name = SUBTYPES.get(subtype, f"UNKNOWN({subtype:#04x})")
        print(f"  sub-type: {subtype:#04x} = {name}")
        if subtype in (0x00, 0x04):
            parse_live(data)
        elif subtype == 0x08:
            parse_settings(data)

# --- BLE reassembly + main loop ---
buf = bytearray()

def on_notify(sender, raw: bytearray):
    global buf
    if len(buf) + len(raw) > 512:
        buf.clear()
    buf.extend(raw)

    pos = 0
    while pos + 3 <= len(buf):
        if buf[pos:pos+3] != HEADER:
            pos += 1
            continue
        if pos + 4 > len(buf):
            break
        length_byte = buf[pos + 3]
        pkt_len = 3 + 1 + length_byte
        if pos + pkt_len > len(buf):
            break
        packet = bytes(buf[pos:pos + pkt_len])
        parse_packet(packet)
        pos += pkt_len

    if pos > 0:
        del buf[:pos]

async def scan_devices(timeout=6):
    from bleak import BleakScanner
    print(f"Сканирую BLE устройства ({timeout}с)...")
    devices = await BleakScanner.discover(timeout=timeout)
    devices.sort(key=lambda d: d.name or "")
    return devices

def is_wheel(name):
    if not name:
        return False
    n = name.lower()
    return any(k in n for k in KNOWN_NAMES)

async def main():
    from bleak import BleakClient

    arg = sys.argv[1] if len(sys.argv) > 1 else None

    if arg == "--scan":
        devices = await scan_devices()
        print(f"\nНайдено {len(devices)} устройств:")
        for d in devices:
            marker = " <-- колесо?" if is_wheel(d.name) else ""
            print(f"  {d.address}  {d.name or '(no name)'}{marker}")
        return

    # Найти устройство
    devices = await scan_devices()

    candidates = []
    if arg:
        # Ищем по подстроке в имени (аргумент командной строки)
        candidates = [d for d in devices if d.name and arg.lower() in d.name.lower()]
        if not candidates:
            print(f"Устройство с именем '{arg}' не найдено. Все устройства:")
            for d in devices:
                print(f"  {d.address}  {d.name or '(no name)'}")
            return
    else:
        # Ищем по известным именам
        candidates = [d for d in devices if is_wheel(d.name)]

    if not candidates:
        print("Колесо не найдено. Все видимые устройства:")
        for d in devices:
            print(f"  {d.address}  {d.name or '(no name)'}")
        print(f"\nЗапусти с именем вручную: python3 {sys.argv[0]} \"имя устройства\"")
        return

    if len(candidates) == 1:
        target = candidates[0]
    else:
        print("Найдено несколько кандидатов:")
        for i, d in enumerate(candidates):
            print(f"  [{i}] {d.address}  {d.name}")
        choice = input("Выбери номер: ").strip()
        target = candidates[int(choice)]

    print(f"\nПодключаюсь к '{target.name}' ({target.address}) ...")

    async with BleakClient(target) as client:
        print("Подключено. Характеристики с notify:")
        for s in client.services:
            for c in s.characteristics:
                if "notify" in c.properties:
                    print(f"  {s.uuid} / {c.uuid}  props={c.properties}")

        await client.start_notify(CHAR_UUID, on_notify)
        print(f"\nСлушаю {CHAR_UUID}. Ctrl+C для выхода.\n")
        try:
            while True:
                await asyncio.sleep(1)
        except KeyboardInterrupt:
            pass
        await client.stop_notify(CHAR_UUID)

if __name__ == "__main__":
    asyncio.run(main())
