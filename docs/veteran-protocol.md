# Veteran / NOSFET BLE Protocol

Протокол BLE для моноколёс Leaperkim Veteran и NOSFET. Реализован в `my_components/veteran/`.

## Источники

- [Таблица разбора пакетов (Google Sheets)](https://docs.google.com/spreadsheets/d/1AFi8-H1jUv8p0RsQlEnBhRtjO1Obn8OK9MPmcfamgvE/edit?usp=sharing)
- Официальное приложение: `com.laoniao.leaperkim` (тот же пакет у NOSFET). Декомпиляция — разд. 9.2; сверены сборки NOSFET 1.1.3 и последняя `com.laoniao.leaperkim`. Старая версия 1.2.1 (`com.lk.lktech`) — разд. 9.1.
- [EUC Charging Monitor](https://github.com/githuba42r/euc_charging) — декодер Veteran, [PROTOCOL_DIAGRAMS.md](https://github.com/githuba42r/euc_charging/blob/eb8c68ba1e1609204928a286723bd10391499e62/PROTOCOL_DIAGRAMS.md)
- [FreeWheel](https://github.com/nathan234/FreeWheel) — KMP-приложение, декодер Veteran
- [Wheellog.Android](https://github.com/Wheellog/Wheellog.Android) — VeteranAdapter

---

## Подключение

- **BLE сервис**: `FFE0`
- **Характеристика**: `FFE1` (notify + write)
- Колесо само шлёт пакеты телеметрии; команды отправляются туда же

---

## Структура входящего пакета

```
[0..2]  Header:      DC 5A 5C
[3]     Length byte: размер оставшейся части (payload + 4 байта CRC)
[4..]   Payload
[last 4] CRC32 LE, big-endian (esp_crc32_le над байтами [0..size-5])
```

Минимальный размер полного пакета: **36 байт** (3 + 1 + 32).

### CRC32

Алгоритм: `esp_crc32_le(0, data, size - 4)`.  
Результат: 4 байта big-endian в конце пакета.

---

## Common Payload (offset 4–35, 32 байта)

Присутствует во всех пакетах.

| Offset | Размер | Поле | Масштаб | Примечание |
|--------|--------|------|---------|------------|
| +0 | u16 BE | voltage | ÷100 → V | |
| +2 | s16 BE | speed | raw | |
| +4 | u32 mid-LE | mileage_current | ÷1000 → km | bytes: [p+6][p+7][p+4][p+5] |
| +8 | u32 mid-LE | mileage_total | ÷1000 → km | |
| +12 | s16 BE | phase_current | raw | |
| +14 | u16 BE | temperature_motor | ÷100 → °C | |
| +16 | u16 BE | auto_off | сек; ≥900 → выключен | |
| +19 | u8 | charging | 0x01 = заряжается | |
| +20 | u16 BE | speed_alert | raw | |
| +22 | u16 BE | speed_tiltback | raw | |
| +24 | u16 BE | fw | modelVersion = fw/1000 | формат: `M.m.pp` |
| +26 | u16 BE | pedals_mode | -100 | |
| +28 | s16 BE | pitch_angle | raw | |
| +30 | u16 BE | pwm | raw | |

**Версия прошивки**: `snprintf("%d.%01d.%02d", fw/1000, (fw/100)%10, fw%100)`

**modelVersion** определяет наличие Extended Payload: при `modelVersion >= 5` и `size >= 47` следует расширенный блок.

---

## Extended Payload (offset 46+)

Байт `[46]` — sub-type. Определяет содержимое пакета.

### Sub-types

| Sub-type | Константа | Описание |
|----------|-----------|----------|
| `0x00` | `SUBTYPE_LIVE` | Live-данные: контроллер, BMS токи, фара |
| `0x04` | `SUBTYPE_LIVE_ALT` | То же, альтернативный тип |
| `0x01` | `SUBTYPE_BMS_LEFT_CELLS_1_15` | BMS left, ячейки 1–15 |
| `0x02` | `SUBTYPE_BMS_LEFT_CELLS_16_30` | BMS left, ячейки 16–30 |
| `0x03` | `SUBTYPE_BMS_LEFT_TEMPS_CELLS_31_36` | BMS left, температуры + ячейки 31–36 |
| `0x05` | `SUBTYPE_BMS_RIGHT_CELLS_1_15` | BMS right, ячейки 1–15 |
| `0x06` | `SUBTYPE_BMS_RIGHT_CELLS_16_30` | BMS right, ячейки 16–30 |
| `0x07` | `SUBTYPE_BMS_RIGHT_TEMPS_CELLS_31_36` | BMS right, температуры + ячейки 31–36 |
| `0x08` | `SUBTYPE_SETTINGS` | Настройки: фара, режимы, uставки |

### Live (0x00 / 0x04) — минимум 73 байта

| Offset | Поле | Масштаб |
|--------|------|---------|
| 59 | temperature_controller | u16 BE ÷100 → °C |
| 69 | bms.left.current | s16 BE ÷ -100 → A |
| 70 | headlight_level | u8: 128=выкл, 0/1/3/5=уровни |
| 71 | bms.right.current | s16 BE ÷ -100 → A |

**Headlight level → текст**: `128` или `≥64` → `"Off"`, `1` → `"Level 1"`, `3` → `"Level 2"`, `5` → `"Level 3"`.

### BMS Cells (0x01/0x02/0x05/0x06) — 15 ячеек

| Offset | Поле | Масштаб |
|--------|------|---------|
| 53 + i×2 | cells[offset + i], i=0..14 | u16 BE ÷1000 → V |

- `0x01` / `0x05`: cell_offset=0 (ячейки 0–14)
- `0x02` / `0x06`: cell_offset=15 (ячейки 15–29)

### BMS Temps + Cells 31–36 (0x03/0x07) — минимум 71 байта

| Offset | Поле | Масштаб |
|--------|------|---------|
| 47 + i×2 | temps[i], i=0..5 | u16 BE ÷100 → °C |
| 59 + i×2 | cells[30+i], i=0..5 | u16 BE ÷1000 → V |
| 71 + i×2 | cells[36+i], i=0..5 | u16 BE ÷1000 → V (42S, size≥83) |

### Settings (0x08) — минимум 67 байт

| Offset | Поле | Значение |
|--------|------|---------|
| 47 | headlight | 0x01 = включена |
| 60 | low_power_mode | 0x01 = включён |
| 61 | high_speed_mode | 0x01 = включён |
| 62 | cut_off_angle | raw |
| 63 | charging_stop_voltage (raw) | u16 BE |
| 66 | tho_ra | u8, % |

**Декодирование charging_stop_voltage**:
```
charging_stop_voltage = read_u16_be(data + 63) + charge_stop_voltage_offset
```
`charge_stop_voltage_offset`: 682 для Veteran Lynx (36S), 0 для NOSFET Aero (30S).  
Финальное значение в V: `charging_stop_voltage / 10.0`.

---

## Исходящие команды

Все команды отправляются в одну и ту же характеристику `FFE1`.

### Структура команды настроек

Команды отправляются как один или два «чанка». Каждый чанк:
```
[N байт payload] [4 байта CRC32 LE big-endian]
```

CRC считается: `esp_crc32_le(0, payload, N)`, результат — big-endian.

### Фара

**Включить:**
```
Chunk 1: 4C 6B 41 70 0D 01 80 80 01 + CRC32
Chunk 2: 4C 64 41 70 0D 01 00 80 01 + CRC32
```

**Выключить:**
```
Chunk 1: 4C 6B 41 70 0D 01 80 80 00 + CRC32
Chunk 2: 4C 64 41 70 0D 01 00 80 00 + CRC32
```

Итого 26 байт (9 + 4 + 9 + 4).

### Максимальное напряжение зарядки

```
Payload (25 байт):
  4C 64 41 70 1D 01 02
  80 80 80 80 80 80 80 80 80 80 80
  80 80 80 80 80 80
  [byte24]
+ CRC32 (4 байта)
```

`byte24 = clamp((voltage - charge_voltage_offset) × 10, 0, 255)`

`charge_voltage_offset` по умолчанию 145.0 (задаётся параметром `charge_voltage_offset` компонента).

**Примеры:**

| Voltage | byte24 | Полный пакет (hex) |
|---------|--------|-------------------|
| 147.0V  | 20 (0x14) | `4C 64 ... 14` + CRC |
| 151.2V  | 62 (0x3E) | `4C 64 ... 3E` + CRC |
| 151.2V (32-bit) | — | `4C 64 41 70 1D 01 02 80 ... 3E 83 26 35 D8` |

Готовые пакеты для 151.2V и 147.0V зашиты в `package-veteran-device.yaml` в switch "Charge to 151.2V".

---

## Расчёт заряда батареи

Линейная интерполяция между `v_empty` и `v_full`:

```
v_full  = nominal_voltage
v_empty = nominal_voltage × (3.0 / 4.2)
pct     = (v - v_empty) / (v_full - v_empty) × 100, clamp 0..100
```

| Конфигурация | nominal_voltage | v_empty |
|---|---|---|
| Veteran Lynx (36S) | 151.2 V | 108.0 V |
| NOSFET Aero (30S) | 126.0 V | 90.0 V |

---

## Конфигурация компонента

```yaml
veteran:
  - id: veteran_lynx
    id_prefix: lynx
    device_id: lynx_device
    sorting_group_id: "sorting_group_lynx"
    sorting_group_id_bms: "sorting_group_lynx_bms"
    nominal_voltage: 151.2       # V полного заряда
    cell_count: 36               # количество ячеек (для min/max/delta)
    charge_stop_voltage_offset: 682  # Lynx: 682, Aero: 0
    # charge_voltage_offset: 145.0  # опционально, дефолт 145.0
    max_charging_voltage_id: max_charging_voltage_lynx
    switch_lights_id: euc_lights_lynx
```

Все сенсоры создаются автоматически через `__init__.py`. Переопределить или скрыть отдельный сенсор можно прямо в блоке `veteran:`.
