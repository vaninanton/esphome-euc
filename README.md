# ESPHome EUC — интеграция моноколеса с умным домом

Проект подключает моноколёса Veteran/NOSFET и зарядное устройство Fast charger к Home Assistant через ESP32 и ESPHome по BLE.

## Возможности

- Подключение до двух моноколёс одновременно (например, Veteran Lynx + NOSFET Aero)
- Мониторинг зарядного устройства Fast charger в реальном времени
- Телеметрия: скорость, напряжение, заряд, температуры, пробег, BMS
- Управление: фара, максимальное напряжение зарядки
- Веб-интерфейс ESPHome с группировкой по устройствам
- Интеграция с Home Assistant через нативный ESPHome API

## Что нужно

- Плата ESP32 (например, ESP32 DevKit v1)
- ESPHome 2025.x или новее
- BLE MAC-адрес каждого устройства

## Быстрый старт

### 1. Клонировать репозиторий

```bash
git clone git@github.com:vaninanton/esphome-euc.git
cd esphome-euc
```

### 2. Найти BLE MAC-адреса устройств

Нужен реальный MAC вида `XX:XX:XX:XX:XX:XX`. Варианты:

**macOS — системный лог (без доп. инструментов)**
```bash
sudo log stream --predicate 'subsystem == "com.apple.bluetooth"' --info | grep -i "adv-addr"
```
Ищи строку вида `adv-addr: 88:56:A6:57:B4:F2-Public, ... devicename: Veteran`.

**Android — приложение nRF Connect**
Scan → найди устройство по имени → MAC под названием устройства.

**Linux**
```bash
bluetoothctl
scan on
```

**ESP32 BLE-сканер (универсально)**
```bash
esphome run esphome-euc-ble-mac-scanner.yaml
```
ESP32 увидит все BLE-устройства поблизости и выведет MAC в логи.

### 3. Заполнить secrets.yaml

```bash
cp secrets.yaml.example secrets.yaml
```

```yaml
wifi_ssid: "ТвояСеть"
wifi_password: "ПарольОтWiFi"
ha_encryption_key: "КлючИзHomeAssistant"
euc_veteran_mac: "XX:XX:XX:XX:XX:XX"   # Veteran Lynx
euc_nosfet_mac: "XX:XX:XX:XX:XX:XX"    # NOSFET Aero
fast_charger_mac: "XX:XX:XX:XX:XX:XX" # Fast charger
```

### 4. Прошить ESP32

```bash
esphome run esphome-euc-veteran.yaml
```

После прошивки ESP32 подключится к Wi-Fi, затем автоматически установит BLE-соединения со всеми устройствами.

---

## Структура проекта

```
esphome-euc-veteran.yaml       — основной конфиг, точка входа
package-veteran-device.yaml    — пакет одного моноколеса Veteran/NOSFET
package-charger-device.yaml    — пакет зарядного устройства Fast charger
my_components/veteran/         — кастомный ESPHome-компонент (C++)
docs/                          — протоколы BLE и описание устройств
secrets.yaml.example           — шаблон secrets
```

## Сенсоры и сущности

### Моноколесо (Veteran Lynx / NOSFET Aero)

| Сущность | Тип | Описание |
|---|---|---|
| Voltage | sensor | Напряжение батареи, V |
| Battery | sensor | Заряд, % (линейная интерполяция) |
| Power | sensor | Мощность (V × I_left + I_right), W |
| BMS Left/Right Current | sensor | Ток каждой половины BMS, A |
| Temperature motor | sensor | Температура мотора, °C |
| Temperature controller | sensor | Температура контроллера, °C |
| Mileage current | sensor | Пробег за поездку, km |
| Mileage total | sensor | Общий пробег, km |
| BMS Cell Voltage Min/Max/Delta | sensor | Мин/макс/разброс ячеек, V |
| BMS Temperature Min/Max | sensor | Мин/макс по 12 датчикам BMS, °C |
| BMS Left/Right Temperature 1–6 | sensor | Все 12 температур BMS, °C |
| Auto Off | sensor | Таймер автовыключения, сек (NaN = выключен) |
| Tho_ra | sensor | Наклон педалей, % |
| Firmware Version | text_sensor | Версия прошивки колеса |
| Headlight | text_sensor | Уровень фары: Off / Level 1 / Level 2 / Level 3 |
| Charging | binary_sensor | Идёт зарядка |
| Connected | binary_sensor | BLE подключено |
| Lights | switch | Управление фарой |
| Max charging voltage | number | Максимальное напряжение зарядки, V |

### Зарядное устройство Fast charger

| Сущность | Тип | Описание |
|---|---|---|
| Out Voltage / Out Current / Out Power | sensor | Выходные параметры |
| In Voltage / In Current / Frequency | sensor | Входные параметры сети |
| Temp 1 / Temp 2 | sensor | Температуры корпуса, °C |
| Efficiency | sensor | КПД, % |
| Charge Ah / Charge Wh | sensor | Накопленный заряд сессии |
| Set Voltage / Set Current | sensor | Текущие уставки устройства (читаются при подключении) |
| Charging | binary_sensor | Идёт зарядка |
| Connected | binary_sensor | BLE подключено |
| Charge | switch | Включить/выключить зарядку |
| Auto-Stop | switch | Автостоп при полном заряде |
| Target Voltage | number | Целевое напряжение, V (синхронизируется с устройством) |
| Target Current | number | Целевой ток, A (синхронизируется с устройством) |

---

## Документация протоколов

- [docs/veteran-protocol.md](docs/veteran-protocol.md) — BLE-протокол Veteran/NOSFET
- [docs/CHARGER.md](docs/CHARGER.md) — BLE-протокол Fast charger
- [docs/inmotion-protocol.md](docs/inmotion-protocol.md) — BLE-протокол Inmotion (справочно)

---

## Добавление нового устройства

Чтобы добавить ещё одно моноколесо Veteran/NOSFET — добавь новый `!include` блок в `esphome-euc-veteran.yaml`:

```yaml
packages:
  - !include
    file: package-veteran-device.yaml
    vars:
      euc_id: mywheel
      euc_name: "My Wheel"
      euc_mac: !secret euc_mywheel_mac
      euc_device_id: mywheel_device
      euc_nominal_voltage: 100.8   # 24S × 4.2V
      euc_cell_count: 24
      euc_charge_voltage_min: 96.0
      euc_charge_stop_offset: 0
```

И добавь устройство и sorting_group в `esphome:` / `web_server:` секции.

---

Pull request'ы приветствуются, особенно если у тебя другая модель и хочешь добавить поддержку.
