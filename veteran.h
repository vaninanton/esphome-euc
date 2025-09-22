#include "esphome.h"

void parse_veteran_packet(std::vector<uint8_t> x)
{
    // ESP_LOGD("PACKET", "%s", format_hex_pretty(x).c_str());
    // ESP_LOGI("P", "%s", format_hex_pretty(x).c_str());

    // Закидываю в буфер всё, что пришло
    for (auto i : x)
    {
        id(ble_buffer).push_back(i);
    }

    if (id(ble_buffer).size() >= 3)
    {
        if (!(id(ble_buffer)[0] == 0xDC && id(ble_buffer)[1] == 0x5A && id(ble_buffer)[2] == 0x5C))
        {
            id(ble_buffer).erase(id(ble_buffer).begin());
        }
    }

    // Поиск начала пакета
    while (id(ble_buffer).size() >= 3)
    {
        if (!(id(ble_buffer)[0] == 0xDC && id(ble_buffer)[1] == 0x5A && id(ble_buffer)[2] == 0x5C))
        {
            id(ble_buffer).erase(id(ble_buffer).begin());
            continue;
        }

        // Минимальный размер заголовка + длина (поставь нужную)
        if (id(ble_buffer).size() < 4)
            break;

        // Если длина ещё не определена — вытащи из заголовка
        if (id(expected_length) == 0)
        {
            id(expected_length) = id(ble_buffer)[3] + 3;
        }

        // Ждём полный пакет
        if (id(ble_buffer).size() < id(expected_length))
            break;

        // Есть полный пакет
        std::vector<uint8_t> bytes(id(ble_buffer).begin(), id(ble_buffer).begin() + id(expected_length));

        // Очистка
        id(ble_buffer).erase(id(ble_buffer).begin(), id(ble_buffer).begin() + id(expected_length));
        id(expected_length) = 0;

        // Тут можно распарсить нужные значения и вернуть их
        ESP_LOGI("P", "%s", format_hex_pretty(bytes).c_str());

        uint16_t voltage = ((bytes[4] & 0xFF) << 8) | (bytes[5] & 0xFF);
        int16_t speed = (bytes[6] << 8) | (bytes[7] & 0xFF);
        int32_t distance = (bytes[8 + 2] << 24) | (bytes[8 + 3] << 16) | (bytes[8] << 8) | bytes[8 + 1];
        int32_t totalDistance = (bytes[12 + 2] << 24) | (bytes[12 + 3] << 16) | (bytes[12] << 8) | bytes[12 + 1];
        int16_t phaseCurrent = (bytes[16] << 8) | (bytes[17] & 0xFF);
        int16_t temperature = (bytes[18] << 8) | (bytes[19] & 0xFF);
        int16_t pitchAngle = (bytes[32] << 8) | (bytes[33] & 0xFF);

        int16_t autoOffSec = ((bytes[20] & 0xFF) << 8) | (bytes[21] & 0xFF);
        int16_t chargeMode = ((bytes[22] & 0xFF) << 8) | (bytes[23] & 0xFF);
        int16_t speedAlert = ((bytes[24] & 0xFF) << 8) | (bytes[25] & 0xFF);
        int16_t speedTiltback = ((bytes[26] & 0xFF) << 8) | (bytes[27] & 0xFF);
        // float pedalsMode = ((bytes[30] & 0xFF) << 8) | (bytes[31] & 0xFF);
        int16_t hwPwmValue = ((bytes[34] & 0xFF) << 8) | (bytes[35] & 0xFF);

        int16_t firmwareVersionBytes = (bytes[28] << 8) | bytes[29];
        char firmwareVersion[16];
        snprintf(firmwareVersion, sizeof(firmwareVersion), "%03d.%01d.%02d", firmwareVersionBytes / 1000, (firmwareVersionBytes % 1000) / 100, (firmwareVersionBytes % 100));

        float voltageValue = voltage / 100.0f;
        float minVoltage = 104.4;
        float maxVoltage = 151.2;
        float batteryValue;

        struct Level { float voltage; float percent; };

        Level levels[] = {
            {151.1, 100},
            {147.3, 95}, {145.9, 90},
            {144.5, 85}, {142.5, 80},
            {140.4, 75}, {138.6, 70},
            {136.8, 65}, {135.1, 60},
            {133.8, 55}, {132.5, 50},
            {131.1, 45}, {129.6, 40},
            {128.0, 35}, {126.4, 30},
            {124.8, 25}, {123.0, 20},
            {121.2, 15}, {119.1, 10},
            {116.2,  5},  {113.4, 0},
            {111.3, -5}, {109.2, -10},
            {108.0, -15}, {106.6, -20},
            {104.4, -25}
        };

        if (voltageValue < minVoltage) {
            batteryValue = -25;
        } else if (voltageValue > maxVoltage) {
            batteryValue = 100;
        } else {
            batteryValue = levels[sizeof(levels)/sizeof(Level)-1].percent;
            for (auto &level : levels) {
                if (voltageValue >= level.voltage) {
                    batteryValue = level.percent;
                    break;
                }
            }
        }

        float speedValue = (speed * 10) / 100.0f;
        float distanceValue = distance / 1000.0f;
        float totalDistanceValue = totalDistance / 1000.0f;
        float phaseCurrentValue = phaseCurrent * 10;
        float temperatureValue = temperature / 100.0f;
        bool chargingValue = chargeMode == 1;
        float autoOffSecValue = autoOffSec / 60.0f;
        float speedAlertValue = speedAlert / 10.0f;
        float speedTiltbackValue = speedTiltback / 10.0f;
        float pitchAngleValue = pitchAngle / 100.0f;

        if (id(euc_voltage).state != voltageValue)
        {
            id(euc_voltage).publish_state(voltageValue);
        }
        
        if (id(euc_battery_percentage).state != batteryValue)
        {
            id(euc_battery_percentage).publish_state(batteryValue);
        }

        // if (id(euc_speed).state != speedValue)
        // {
        //     id(euc_speed).publish_state(speedValue);
        // }

        // if (id(euc_distance).state != distanceValue)
        // {
        //     id(euc_distance).publish_state(distanceValue);
        // }

        if (id(euc_total_mileage).state != totalDistanceValue)
        {
            id(euc_total_mileage).publish_state(totalDistanceValue);
        }

        // if (id(euc_phase_current).state != phaseCurrentValue)
        // {
        //     id(euc_phase_current).publish_state(phaseCurrentValue);
        // }

        if (id(euc_temperature).state != temperatureValue)
        {
            id(euc_temperature).publish_state(temperatureValue);
        }

        if (id(euc_auto_off_sec).state != autoOffSecValue)
        {
            id(euc_auto_off_sec).publish_state(autoOffSecValue);
        }

        id(euc_charging).publish_state(chargingValue);

        // if (id(euc_speed_alert).state != speedAlertValue)
        // {
        //     id(euc_speed_alert).publish_state(speedAlertValue);
        // }

        // if (id(euc_speed_tiltback).state != speedTiltbackValue)
        // {
        //     id(euc_speed_tiltback).publish_state(speedTiltbackValue);
        // }

        if (id(euc_pitch_angle).state != pitchAngleValue)
        {
            id(euc_pitch_angle).publish_state(pitchAngleValue);
        }

        // if (id(euc_hw_pwm).state != hwPwmValue)
        // {
        //     id(euc_hw_pwm).publish_state(hwPwmValue);
        // }

        if (id(euc_firmware_version).state != firmwareVersion)
        {
            id(euc_firmware_version).publish_state(firmwareVersion);
        }
    }
}
