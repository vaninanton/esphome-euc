#include "esphome.h"

void parse_inmotionv2_packet(std::vector<uint8_t> x)
{
    // ESP_LOGD("PACKET", "%s", format_hex_pretty(x).c_str());
    // ESP_LOGI("P", "%s", format_hex_pretty(x).c_str());
    if (x[2] != 0x14 || (x[4] & 0x7F) != 0x04) {
        return {};
    }

    bool isCharging = (x[61] >> 7) & 0x01;
    bool isLifted = (x[61] >> 6) & 0x01;
    uint16_t batteryPercentage = ((x[34] & 0xFF) << 8) | (x[33] & 0xFF);
    uint16_t voltageValue = ((x[6] & 0xFF) << 8) | (x[5] & 0xFF);
    int16_t currentValue = ((x[8] << 8) | (x[7] & 0xFF));
    float powerValue = (currentValue / 100.0f) * (voltageValue / 100.0f);

    id(euc_charging).publish_state(isCharging);
    id(euc_lifted).publish_state(isLifted);
    id(euc_battery_percentage).publish_state(batteryPercentage / 100.0f);
    id(euc_voltage).publish_state(voltageValue / 100.0f);
    id(euc_current).publish_state(currentValue / 100.0f);
    id(euc_power).publish_state(powerValue);
}