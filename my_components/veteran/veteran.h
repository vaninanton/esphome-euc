#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esp_crc.h"

namespace esphome
{
namespace veteran
{
struct BMSBlockData
{
  float cell01;
  float cell02;
  float cell03;
  float cell04;
  float cell05;
  float cell06;
  float cell07;
  float cell08;
  float cell09;
  float cell10;
  float cell11;
  float cell12;
  float cell13;
  float cell14;
  float cell15;
  float cell16;
  float cell17;
  float cell18;
  float cell19;
  float cell20;
  float cell21;
  float cell22;
  float cell23;
  float cell24;
  float cell25;
  float cell26;
  float cell27;
  float cell28;
  float cell29;
  float cell30;
  float cell31;
  float cell32;
  float cell33;
  float cell34;
  float cell35;
  float cell36;
  float current;
  float temp1;
  float temp2;
  float temp3;
  float temp4;
  float temp5;
  float temp6;
};

struct BMSData
{
  BMSBlockData left;
  BMSBlockData right;
};

struct EUCData
{
  bool charging;
  bool high_speed_mode;
  bool low_power_mode;
  char firmware_version[9];
  float mileage_current; // Пробег текущей поездки
  float mileage_total;   // Общий пробег
  float temperature_controller;
  float temperature_motor;
  uint16_t auto_off; // Время в секундах до выключения (900 - выключаться не будет)
  uint16_t brightness;
  uint16_t charging_stop_voltage;
  uint16_t current;
  uint16_t cut_off_angle;
  uint16_t fw;
  uint16_t gyro_level;
  uint16_t modelVersion;
  uint16_t pedals_mode; // 0 - soft, 100 - hard
  uint16_t phase_current;
  uint16_t pitch_angle;
  uint16_t pwm;
  uint16_t speed;
  uint16_t speed_alert;
  uint16_t speed_tiltback;
  uint16_t tho_ra;
  uint16_t voltage;

  float battery_percentage(bool linearType = false) const
  {
    if (linearType)
    {
      if (this->voltage > 14804)
        return 100;
      if (this->voltage > 11903)
        return (this->voltage - 11902) / 29.03;
      return 0;
    }

    if (this->voltage > 15030)
      return 100;
    if (this->voltage > 12240)
      return (this->voltage - 11970) / 30.6;
    if (this->voltage > 11520)
      return (this->voltage - 11520) / 81.0;
    return 0;
  }

  BMSData bms;
};

class VeteranComponent : public Component
{
public:
  EUCData euc;

  void parse_ble_packet(const std::vector<uint8_t> &x);
  void parse_packet(const std::vector<uint8_t> &bytes);
  void on_ble_disconnected();

  void binary_sensor_charging(binary_sensor::BinarySensor *s) { binary_sensor_charging_ = s; }
  void binary_sensor_low_power_mode(binary_sensor::BinarySensor *s) { binary_sensor_low_power_mode_ = s; }
  void binary_sensor_high_speed_mode(binary_sensor::BinarySensor *s) { binary_sensor_high_speed_mode_ = s; }
  void sensor_auto_off(sensor::Sensor *s) { sensor_auto_off_ = s; }
  void sensor_battery_percentage(sensor::Sensor *s) { sensor_battery_percentage_ = s; }
  void sensor_bms_left_current(sensor::Sensor *s) { sensor_bms_left_current_ = s; }
  void sensor_bms_right_current(sensor::Sensor *s) { sensor_bms_right_current_ = s; }
  void sensor_charging_stop_voltage(sensor::Sensor *s) { sensor_charging_stop_voltage_ = s; }

  void sensor_bms_left_cell_01(sensor::Sensor *s) { sensor_bms_left_cell_01_ = s; }
  void sensor_bms_left_cell_02(sensor::Sensor *s) { sensor_bms_left_cell_02_ = s; }
  void sensor_bms_left_cell_03(sensor::Sensor *s) { sensor_bms_left_cell_03_ = s; }
  void sensor_bms_left_cell_04(sensor::Sensor *s) { sensor_bms_left_cell_04_ = s; }
  void sensor_bms_left_cell_05(sensor::Sensor *s) { sensor_bms_left_cell_05_ = s; }
  void sensor_bms_left_cell_06(sensor::Sensor *s) { sensor_bms_left_cell_06_ = s; }
  void sensor_bms_left_cell_07(sensor::Sensor *s) { sensor_bms_left_cell_07_ = s; }
  void sensor_bms_left_cell_08(sensor::Sensor *s) { sensor_bms_left_cell_08_ = s; }
  void sensor_bms_left_cell_09(sensor::Sensor *s) { sensor_bms_left_cell_09_ = s; }
  void sensor_bms_left_cell_10(sensor::Sensor *s) { sensor_bms_left_cell_10_ = s; }
  void sensor_bms_left_cell_11(sensor::Sensor *s) { sensor_bms_left_cell_11_ = s; }
  void sensor_bms_left_cell_12(sensor::Sensor *s) { sensor_bms_left_cell_12_ = s; }
  void sensor_bms_left_cell_13(sensor::Sensor *s) { sensor_bms_left_cell_13_ = s; }
  void sensor_bms_left_cell_14(sensor::Sensor *s) { sensor_bms_left_cell_14_ = s; }
  void sensor_bms_left_cell_15(sensor::Sensor *s) { sensor_bms_left_cell_15_ = s; }
  void sensor_bms_left_cell_16(sensor::Sensor *s) { sensor_bms_left_cell_16_ = s; }
  void sensor_bms_left_cell_17(sensor::Sensor *s) { sensor_bms_left_cell_17_ = s; }
  void sensor_bms_left_cell_18(sensor::Sensor *s) { sensor_bms_left_cell_18_ = s; }
  void sensor_bms_left_cell_19(sensor::Sensor *s) { sensor_bms_left_cell_19_ = s; }
  void sensor_bms_left_cell_20(sensor::Sensor *s) { sensor_bms_left_cell_20_ = s; }
  void sensor_bms_left_cell_21(sensor::Sensor *s) { sensor_bms_left_cell_21_ = s; }
  void sensor_bms_left_cell_22(sensor::Sensor *s) { sensor_bms_left_cell_22_ = s; }
  void sensor_bms_left_cell_23(sensor::Sensor *s) { sensor_bms_left_cell_23_ = s; }
  void sensor_bms_left_cell_24(sensor::Sensor *s) { sensor_bms_left_cell_24_ = s; }
  void sensor_bms_left_cell_25(sensor::Sensor *s) { sensor_bms_left_cell_25_ = s; }
  void sensor_bms_left_cell_26(sensor::Sensor *s) { sensor_bms_left_cell_26_ = s; }
  void sensor_bms_left_cell_27(sensor::Sensor *s) { sensor_bms_left_cell_27_ = s; }
  void sensor_bms_left_cell_28(sensor::Sensor *s) { sensor_bms_left_cell_28_ = s; }
  void sensor_bms_left_cell_29(sensor::Sensor *s) { sensor_bms_left_cell_29_ = s; }
  void sensor_bms_left_cell_30(sensor::Sensor *s) { sensor_bms_left_cell_30_ = s; }
  void sensor_bms_left_cell_31(sensor::Sensor *s) { sensor_bms_left_cell_31_ = s; }
  void sensor_bms_left_cell_32(sensor::Sensor *s) { sensor_bms_left_cell_32_ = s; }
  void sensor_bms_left_cell_33(sensor::Sensor *s) { sensor_bms_left_cell_33_ = s; }
  void sensor_bms_left_cell_34(sensor::Sensor *s) { sensor_bms_left_cell_34_ = s; }
  void sensor_bms_left_cell_35(sensor::Sensor *s) { sensor_bms_left_cell_35_ = s; }
  void sensor_bms_left_cell_36(sensor::Sensor *s) { sensor_bms_left_cell_36_ = s; }

  void sensor_bms_right_cell_01(sensor::Sensor *s) { sensor_bms_right_cell_01_ = s; }
  void sensor_bms_right_cell_02(sensor::Sensor *s) { sensor_bms_right_cell_02_ = s; }
  void sensor_bms_right_cell_03(sensor::Sensor *s) { sensor_bms_right_cell_03_ = s; }
  void sensor_bms_right_cell_04(sensor::Sensor *s) { sensor_bms_right_cell_04_ = s; }
  void sensor_bms_right_cell_05(sensor::Sensor *s) { sensor_bms_right_cell_05_ = s; }
  void sensor_bms_right_cell_06(sensor::Sensor *s) { sensor_bms_right_cell_06_ = s; }
  void sensor_bms_right_cell_07(sensor::Sensor *s) { sensor_bms_right_cell_07_ = s; }
  void sensor_bms_right_cell_08(sensor::Sensor *s) { sensor_bms_right_cell_08_ = s; }
  void sensor_bms_right_cell_09(sensor::Sensor *s) { sensor_bms_right_cell_09_ = s; }
  void sensor_bms_right_cell_10(sensor::Sensor *s) { sensor_bms_right_cell_10_ = s; }
  void sensor_bms_right_cell_11(sensor::Sensor *s) { sensor_bms_right_cell_11_ = s; }
  void sensor_bms_right_cell_12(sensor::Sensor *s) { sensor_bms_right_cell_12_ = s; }
  void sensor_bms_right_cell_13(sensor::Sensor *s) { sensor_bms_right_cell_13_ = s; }
  void sensor_bms_right_cell_14(sensor::Sensor *s) { sensor_bms_right_cell_14_ = s; }
  void sensor_bms_right_cell_15(sensor::Sensor *s) { sensor_bms_right_cell_15_ = s; }
  void sensor_bms_right_cell_16(sensor::Sensor *s) { sensor_bms_right_cell_16_ = s; }
  void sensor_bms_right_cell_17(sensor::Sensor *s) { sensor_bms_right_cell_17_ = s; }
  void sensor_bms_right_cell_18(sensor::Sensor *s) { sensor_bms_right_cell_18_ = s; }
  void sensor_bms_right_cell_19(sensor::Sensor *s) { sensor_bms_right_cell_19_ = s; }
  void sensor_bms_right_cell_20(sensor::Sensor *s) { sensor_bms_right_cell_20_ = s; }
  void sensor_bms_right_cell_21(sensor::Sensor *s) { sensor_bms_right_cell_21_ = s; }
  void sensor_bms_right_cell_22(sensor::Sensor *s) { sensor_bms_right_cell_22_ = s; }
  void sensor_bms_right_cell_23(sensor::Sensor *s) { sensor_bms_right_cell_23_ = s; }
  void sensor_bms_right_cell_24(sensor::Sensor *s) { sensor_bms_right_cell_24_ = s; }
  void sensor_bms_right_cell_25(sensor::Sensor *s) { sensor_bms_right_cell_25_ = s; }
  void sensor_bms_right_cell_26(sensor::Sensor *s) { sensor_bms_right_cell_26_ = s; }
  void sensor_bms_right_cell_27(sensor::Sensor *s) { sensor_bms_right_cell_27_ = s; }
  void sensor_bms_right_cell_28(sensor::Sensor *s) { sensor_bms_right_cell_28_ = s; }
  void sensor_bms_right_cell_29(sensor::Sensor *s) { sensor_bms_right_cell_29_ = s; }
  void sensor_bms_right_cell_30(sensor::Sensor *s) { sensor_bms_right_cell_30_ = s; }
  void sensor_bms_right_cell_31(sensor::Sensor *s) { sensor_bms_right_cell_31_ = s; }
  void sensor_bms_right_cell_32(sensor::Sensor *s) { sensor_bms_right_cell_32_ = s; }
  void sensor_bms_right_cell_33(sensor::Sensor *s) { sensor_bms_right_cell_33_ = s; }
  void sensor_bms_right_cell_34(sensor::Sensor *s) { sensor_bms_right_cell_34_ = s; }
  void sensor_bms_right_cell_35(sensor::Sensor *s) { sensor_bms_right_cell_35_ = s; }
  void sensor_bms_right_cell_36(sensor::Sensor *s) { sensor_bms_right_cell_36_ = s; }

  void sensor_bms_left_temp_1(sensor::Sensor *s) { sensor_bms_left_temp_1_ = s; }
  void sensor_bms_left_temp_2(sensor::Sensor *s) { sensor_bms_left_temp_2_ = s; }
  void sensor_bms_left_temp_3(sensor::Sensor *s) { sensor_bms_left_temp_3_ = s; }
  void sensor_bms_left_temp_4(sensor::Sensor *s) { sensor_bms_left_temp_4_ = s; }
  void sensor_bms_left_temp_5(sensor::Sensor *s) { sensor_bms_left_temp_5_ = s; }
  void sensor_bms_left_temp_6(sensor::Sensor *s) { sensor_bms_left_temp_6_ = s; }
  void sensor_bms_right_temp_1(sensor::Sensor *s) { sensor_bms_right_temp_1_ = s; }
  void sensor_bms_right_temp_2(sensor::Sensor *s) { sensor_bms_right_temp_2_ = s; }
  void sensor_bms_right_temp_3(sensor::Sensor *s) { sensor_bms_right_temp_3_ = s; }
  void sensor_bms_right_temp_4(sensor::Sensor *s) { sensor_bms_right_temp_4_ = s; }
  void sensor_bms_right_temp_5(sensor::Sensor *s) { sensor_bms_right_temp_5_ = s; }
  void sensor_bms_right_temp_6(sensor::Sensor *s) { sensor_bms_right_temp_6_ = s; }

  void sensor_temperature_motor(sensor::Sensor *s) { sensor_temperature_motor_ = s; }
  void sensor_temperature_controller(sensor::Sensor *s) { sensor_temperature_controller_ = s; }
  void sensor_tho_ra(sensor::Sensor *s) { sensor_tho_ra_ = s; }
  void sensor_mileage_current(sensor::Sensor *s) { sensor_mileage_current_ = s; }
  void sensor_mileage_total(sensor::Sensor *s) { sensor_mileage_total_ = s; }
  void sensor_voltage(sensor::Sensor *s) { sensor_voltage_ = s; }
  void text_sensor_firmware_version(text_sensor::TextSensor *s) { text_sensor_firmware_version_ = s; }

protected:
  std::vector<uint8_t> ble_buffer_;

  binary_sensor::BinarySensor *binary_sensor_charging_;
  binary_sensor::BinarySensor *binary_sensor_low_power_mode_;
  binary_sensor::BinarySensor *binary_sensor_high_speed_mode_;
  sensor::Sensor *sensor_auto_off_;
  sensor::Sensor *sensor_battery_percentage_;
  sensor::Sensor *sensor_bms_left_current_;
  sensor::Sensor *sensor_bms_right_current_;
  sensor::Sensor *sensor_charging_stop_voltage_;

  sensor::Sensor *sensor_bms_left_cell_01_;
  sensor::Sensor *sensor_bms_left_cell_02_;
  sensor::Sensor *sensor_bms_left_cell_03_;
  sensor::Sensor *sensor_bms_left_cell_04_;
  sensor::Sensor *sensor_bms_left_cell_05_;
  sensor::Sensor *sensor_bms_left_cell_06_;
  sensor::Sensor *sensor_bms_left_cell_07_;
  sensor::Sensor *sensor_bms_left_cell_08_;
  sensor::Sensor *sensor_bms_left_cell_09_;
  sensor::Sensor *sensor_bms_left_cell_10_;
  sensor::Sensor *sensor_bms_left_cell_11_;
  sensor::Sensor *sensor_bms_left_cell_12_;
  sensor::Sensor *sensor_bms_left_cell_13_;
  sensor::Sensor *sensor_bms_left_cell_14_;
  sensor::Sensor *sensor_bms_left_cell_15_;
  sensor::Sensor *sensor_bms_left_cell_16_;
  sensor::Sensor *sensor_bms_left_cell_17_;
  sensor::Sensor *sensor_bms_left_cell_18_;
  sensor::Sensor *sensor_bms_left_cell_19_;
  sensor::Sensor *sensor_bms_left_cell_20_;
  sensor::Sensor *sensor_bms_left_cell_21_;
  sensor::Sensor *sensor_bms_left_cell_22_;
  sensor::Sensor *sensor_bms_left_cell_23_;
  sensor::Sensor *sensor_bms_left_cell_24_;
  sensor::Sensor *sensor_bms_left_cell_25_;
  sensor::Sensor *sensor_bms_left_cell_26_;
  sensor::Sensor *sensor_bms_left_cell_27_;
  sensor::Sensor *sensor_bms_left_cell_28_;
  sensor::Sensor *sensor_bms_left_cell_29_;
  sensor::Sensor *sensor_bms_left_cell_30_;
  sensor::Sensor *sensor_bms_left_cell_31_;
  sensor::Sensor *sensor_bms_left_cell_32_;
  sensor::Sensor *sensor_bms_left_cell_33_;
  sensor::Sensor *sensor_bms_left_cell_34_;
  sensor::Sensor *sensor_bms_left_cell_35_;
  sensor::Sensor *sensor_bms_left_cell_36_;

  sensor::Sensor *sensor_bms_right_cell_01_;
  sensor::Sensor *sensor_bms_right_cell_02_;
  sensor::Sensor *sensor_bms_right_cell_03_;
  sensor::Sensor *sensor_bms_right_cell_04_;
  sensor::Sensor *sensor_bms_right_cell_05_;
  sensor::Sensor *sensor_bms_right_cell_06_;
  sensor::Sensor *sensor_bms_right_cell_07_;
  sensor::Sensor *sensor_bms_right_cell_08_;
  sensor::Sensor *sensor_bms_right_cell_09_;
  sensor::Sensor *sensor_bms_right_cell_10_;
  sensor::Sensor *sensor_bms_right_cell_11_;
  sensor::Sensor *sensor_bms_right_cell_12_;
  sensor::Sensor *sensor_bms_right_cell_13_;
  sensor::Sensor *sensor_bms_right_cell_14_;
  sensor::Sensor *sensor_bms_right_cell_15_;
  sensor::Sensor *sensor_bms_right_cell_16_;
  sensor::Sensor *sensor_bms_right_cell_17_;
  sensor::Sensor *sensor_bms_right_cell_18_;
  sensor::Sensor *sensor_bms_right_cell_19_;
  sensor::Sensor *sensor_bms_right_cell_20_;
  sensor::Sensor *sensor_bms_right_cell_21_;
  sensor::Sensor *sensor_bms_right_cell_22_;
  sensor::Sensor *sensor_bms_right_cell_23_;
  sensor::Sensor *sensor_bms_right_cell_24_;
  sensor::Sensor *sensor_bms_right_cell_25_;
  sensor::Sensor *sensor_bms_right_cell_26_;
  sensor::Sensor *sensor_bms_right_cell_27_;
  sensor::Sensor *sensor_bms_right_cell_28_;
  sensor::Sensor *sensor_bms_right_cell_29_;
  sensor::Sensor *sensor_bms_right_cell_30_;
  sensor::Sensor *sensor_bms_right_cell_31_;
  sensor::Sensor *sensor_bms_right_cell_32_;
  sensor::Sensor *sensor_bms_right_cell_33_;
  sensor::Sensor *sensor_bms_right_cell_34_;
  sensor::Sensor *sensor_bms_right_cell_35_;
  sensor::Sensor *sensor_bms_right_cell_36_;

  sensor::Sensor *sensor_bms_left_temp_1_;
  sensor::Sensor *sensor_bms_left_temp_2_;
  sensor::Sensor *sensor_bms_left_temp_3_;
  sensor::Sensor *sensor_bms_left_temp_4_;
  sensor::Sensor *sensor_bms_left_temp_5_;
  sensor::Sensor *sensor_bms_left_temp_6_;
  sensor::Sensor *sensor_bms_right_temp_1_;
  sensor::Sensor *sensor_bms_right_temp_2_;
  sensor::Sensor *sensor_bms_right_temp_3_;
  sensor::Sensor *sensor_bms_right_temp_4_;
  sensor::Sensor *sensor_bms_right_temp_5_;
  sensor::Sensor *sensor_bms_right_temp_6_;
  sensor::Sensor *sensor_temperature_motor_;
  sensor::Sensor *sensor_temperature_controller_;
  sensor::Sensor *sensor_tho_ra_;
  sensor::Sensor *sensor_mileage_current_;
  sensor::Sensor *sensor_mileage_total_;
  sensor::Sensor *sensor_voltage_;
  text_sensor::TextSensor *text_sensor_firmware_version_;

  uint16_t unsignedShortFromBytesBE(const std::vector<uint8_t> &bytes, int offset)
  {
    if (bytes.empty() || bytes.size() < offset + 2)
    {
      ESP_LOGE("unsignedShortFromBytesBE", "bytes is empty or too short");
      return 0;
    }

    uint8_t high = bytes[offset], low = bytes[offset + 1];
    uint16_t v = high << 8 | low;
    return v;
  }

  int16_t shortFromBytesBE(const std::vector<uint8_t> &bytes, int offset)
  {
    if (bytes.empty() || bytes.size() < offset + 2)
    {
      ESP_LOGE("shortFromBytesBE", "bytes is empty or too short");
      return 0;
    }

    int8_t high = bytes[offset], low = bytes[offset + 1];
    int16_t v = high << 8 | low;
    return v;
  }

  uint32_t longFromBytesBE(const std::vector<uint8_t> &bytes, int offset)
  {
    if (bytes.empty() || bytes.size() < offset + 4)
    {
      ESP_LOGE("longFromBytesBE", "bytes is empty or too short");
      return 0;
    }
    return (((bytes[offset] & 0xFF) << 24) | ((bytes[offset + 1] & 0xFF) << 16) | ((bytes[offset + 2] & 0xFF) << 8) | (bytes[offset + 3] & 0xFF)) & 0xFFFFFFFFL;
  }

  uint32_t unsignedLongFromBytesMidLE(const std::vector<uint8_t> &bytes, int offset)
  {
    if (bytes.empty() || bytes.size() < offset + 4)
    {
      ESP_LOGE("unsignedLongFromBytesMidLE", "bytes is empty or too short");
      return 0;
    }

    return (((bytes[offset + 2] & 0xFF) << 24) | ((bytes[offset + 3] & 0xFF) << 16) | ((bytes[offset] & 0xFF) << 8) | (bytes[offset + 1] & 0xFF));
  }

  bool check_crc32(const std::vector<uint8_t> &bytes, uint8_t expected_length)
  {
    // Считаем CRC по всему, кроме последних двух байт
    uint32_t crc_calc = esp_crc32_le(0, bytes.data(), expected_length);
    uint32_t crc_recv = longFromBytesBE(bytes, expected_length);

    if (crc_recv == crc_calc)
      return true;
    ESP_LOGW("CRC", "(recv=%04X calc=%04X)", crc_recv, crc_calc);
    return false;
  }
};
} // namespace veteran
} // namespace esphome