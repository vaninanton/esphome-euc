// Copyright 2025 <Tony V>
#include <cstdio>
#include <vector>
#include "veteran.h"

namespace esphome {
namespace veteran {

// Чтение из указателя (пакет уже проверен по длине)
static inline uint16_t read_u16_be(const uint8_t *p) { return (uint16_t) ((p[0] << 8) | p[1]); }
static inline int16_t read_s16_be(const uint8_t *p) { return (int16_t) read_u16_be(p); }
static inline uint32_t read_u32_be(const uint8_t *p) {
  return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | p[3];
}
static inline uint32_t read_u32_midle(const uint8_t *p) {
  return ((uint32_t) p[2] << 24) | ((uint32_t) p[3] << 16) | ((uint32_t) p[0] << 8) | p[1];
}

void VeteranComponent::parse_ble_packet(const std::vector<uint8_t> &x) {
  this->ble_buffer_.insert(this->ble_buffer_.end(), x.begin(), x.end());

  constexpr uint8_t HEADER[] = {0xDC, 0x5A, 0x5C};
  constexpr size_t HEADER_SIZE = sizeof(HEADER);

  while (this->ble_buffer_.size() >= HEADER_SIZE) {
    if (!std::equal(HEADER, HEADER + HEADER_SIZE, this->ble_buffer_.begin())) {
      this->ble_buffer_.erase(this->ble_buffer_.begin());
      continue;
    }
    if (this->ble_buffer_.size() < HEADER_SIZE + 1) break;
    uint8_t length = this->ble_buffer_[3];
    size_t expected_length = HEADER_SIZE + 1 + length;
    if (this->ble_buffer_.size() < expected_length) break;

    const uint8_t *p = this->ble_buffer_.data();
    if (check_crc32(p, expected_length))
      parse_packet(p, expected_length);

    this->ble_buffer_.erase(this->ble_buffer_.begin(), this->ble_buffer_.begin() + expected_length);
  }

  if (this->ble_buffer_.size() > 1024) this->ble_buffer_.clear();
}

void VeteranComponent::parse_packet(const std::vector<uint8_t> &bytes) {
  parse_packet(bytes.data(), bytes.size());
}

void VeteranComponent::parse_packet(const uint8_t *data, size_t size) {
  if (size < 36) return;
#define R16(o) read_u16_be(data + (o))
#define R16s(o) read_s16_be(data + (o))
#define R32m(o) read_u32_midle(data + (o))

  this->euc.voltage = R16(4);
  this->euc.speed = R16s(6);
  this->euc.mileage_current = R32m(8) / 1000.0f;
  this->euc.mileage_total = R32m(12) / 1000.0f;
  this->euc.phase_current = R16s(16);
  this->euc.temperature_motor = R16(18) / 100.0f;
  this->euc.auto_off = R16(20);
  this->euc.charging = data[23] == 0x01;
  this->euc.speed_alert = R16(24);
  this->euc.speed_tiltback = R16(26);

  uint16_t fw = R16(28);
  this->euc.modelVersion = fw / 1000;
  snprintf(this->euc.firmware_version, sizeof(this->euc.firmware_version), "%03d.%01d.%02d",
           this->euc.modelVersion, (fw / 100) % 10, fw % 100);

  this->euc.pedals_mode = R16(30) - 100;
  this->euc.pitch_angle = R16s(32);
  this->euc.pwm = R16(34);

  this->sensor_voltage_->publish_state(this->euc.voltage / 100.0f);
  this->sensor_mileage_current_->publish_state(this->euc.mileage_current);
  this->sensor_mileage_total_->publish_state(this->euc.mileage_total);
  this->sensor_temperature_motor_->publish_state(this->euc.temperature_motor);
  if (this->euc.auto_off < 900)
    this->sensor_auto_off_->publish_state(this->euc.auto_off);
  else
    this->sensor_auto_off_->publish_state(NAN);
  this->binary_sensor_charging_->publish_state(this->euc.charging);
  this->text_sensor_firmware_version_->publish_state(this->euc.firmware_version);
  this->sensor_battery_percentage_->publish_state(this->euc.battery_percentage());

  if (this->euc.modelVersion >= 5 && size >= 47) {
    switch (data[46]) {
      case 0x00:
      case 0x04:
        if (size >= 72) {
          this->euc.temperature_controller = R16(59) / 100.0f;
          this->euc.bms.left.current = R16s(69) / -100.0f;
          this->euc.bms.right.current = R16s(71) / -100.0f;
          this->sensor_bms_left_current_->publish_state(this->euc.bms.left.current);
          this->sensor_bms_right_current_->publish_state(this->euc.bms.right.current);
          this->sensor_power_->publish_state(this->euc.voltage * (this->euc.bms.left.current + this->euc.bms.right.current) / 100.0f);
          this->sensor_temperature_controller_->publish_state(this->euc.temperature_controller);
        }
        break;
      case 0x01:
        if (size >= 82) {
          this->euc.bms.left.cell01 = R16(53) / 1000.0f;
          this->euc.bms.left.cell02 = R16(55) / 1000.0f;
          this->euc.bms.left.cell03 = R16(57) / 1000.0f;
          this->euc.bms.left.cell04 = R16(59) / 1000.0f;
          this->euc.bms.left.cell05 = R16(61) / 1000.0f;
          this->euc.bms.left.cell06 = R16(63) / 1000.0f;
          this->euc.bms.left.cell07 = R16(65) / 1000.0f;
          this->euc.bms.left.cell08 = R16(67) / 1000.0f;
          this->euc.bms.left.cell09 = R16(69) / 1000.0f;
          this->euc.bms.left.cell10 = R16(71) / 1000.0f;
          this->euc.bms.left.cell11 = R16(73) / 1000.0f;
          this->euc.bms.left.cell12 = R16(75) / 1000.0f;
          this->euc.bms.left.cell13 = R16(77) / 1000.0f;
          this->euc.bms.left.cell14 = R16(79) / 1000.0f;
          this->euc.bms.left.cell15 = R16(81) / 1000.0f;
        }
        break;

      case 0x02:
        if (size >= 82) {
          this->euc.bms.left.cell16 = R16(53) / 1000.0f;
          this->euc.bms.left.cell17 = R16(55) / 1000.0f;
          this->euc.bms.left.cell18 = R16(57) / 1000.0f;
          this->euc.bms.left.cell19 = R16(59) / 1000.0f;
          this->euc.bms.left.cell20 = R16(61) / 1000.0f;
          this->euc.bms.left.cell21 = R16(63) / 1000.0f;
          this->euc.bms.left.cell22 = R16(65) / 1000.0f;
          this->euc.bms.left.cell23 = R16(67) / 1000.0f;
          this->euc.bms.left.cell24 = R16(69) / 1000.0f;
          this->euc.bms.left.cell25 = R16(71) / 1000.0f;
          this->euc.bms.left.cell26 = R16(73) / 1000.0f;
          this->euc.bms.left.cell27 = R16(75) / 1000.0f;
          this->euc.bms.left.cell28 = R16(77) / 1000.0f;
          this->euc.bms.left.cell29 = R16(79) / 1000.0f;
          this->euc.bms.left.cell30 = R16(81) / 1000.0f;
        }
        break;

      case 0x03:
        if (size >= 70) {
          this->euc.bms.left.temp1 = R16(47) / 100.0f;
          this->euc.bms.left.temp2 = R16(49) / 100.0f;
          this->euc.bms.left.temp3 = R16(51) / 100.0f;
          this->euc.bms.left.temp4 = R16(53) / 100.0f;
          this->euc.bms.left.temp5 = R16(55) / 100.0f;
          this->euc.bms.left.temp6 = R16(57) / 100.0f;
          this->euc.bms.left.cell31 = R16(59) / 1000.0f;
          this->euc.bms.left.cell32 = R16(61) / 1000.0f;
          this->euc.bms.left.cell33 = R16(63) / 1000.0f;
          this->euc.bms.left.cell34 = R16(65) / 1000.0f;
          this->euc.bms.left.cell35 = R16(67) / 1000.0f;
          this->euc.bms.left.cell36 = R16(69) / 1000.0f;
          this->sensor_bms_left_temp_1_->publish_state(this->euc.bms.left.temp1);
          this->sensor_bms_left_temp_2_->publish_state(this->euc.bms.left.temp2);
          this->sensor_bms_left_temp_3_->publish_state(this->euc.bms.left.temp3);
          this->sensor_bms_left_temp_4_->publish_state(this->euc.bms.left.temp4);
          this->sensor_bms_left_temp_5_->publish_state(this->euc.bms.left.temp5);
          this->sensor_bms_left_temp_6_->publish_state(this->euc.bms.left.temp6);
        }
        break;
      case 0x05:
        if (size >= 82) {
          this->euc.bms.right.cell01 = R16(53) / 1000.0f;
          this->euc.bms.right.cell02 = R16(55) / 1000.0f;
          this->euc.bms.right.cell03 = R16(57) / 1000.0f;
          this->euc.bms.right.cell04 = R16(59) / 1000.0f;
          this->euc.bms.right.cell05 = R16(61) / 1000.0f;
          this->euc.bms.right.cell06 = R16(63) / 1000.0f;
          this->euc.bms.right.cell07 = R16(65) / 1000.0f;
          this->euc.bms.right.cell08 = R16(67) / 1000.0f;
          this->euc.bms.right.cell09 = R16(69) / 1000.0f;
          this->euc.bms.right.cell10 = R16(71) / 1000.0f;
          this->euc.bms.right.cell11 = R16(73) / 1000.0f;
          this->euc.bms.right.cell12 = R16(75) / 1000.0f;
          this->euc.bms.right.cell13 = R16(77) / 1000.0f;
          this->euc.bms.right.cell14 = R16(79) / 1000.0f;
          this->euc.bms.right.cell15 = R16(81) / 1000.0f;
        }
        break;

      case 0x06:
        if (size >= 82) {
          this->euc.bms.right.cell16 = R16(53) / 1000.0f;
          this->euc.bms.right.cell17 = R16(55) / 1000.0f;
          this->euc.bms.right.cell18 = R16(57) / 1000.0f;
          this->euc.bms.right.cell19 = R16(59) / 1000.0f;
          this->euc.bms.right.cell20 = R16(61) / 1000.0f;
          this->euc.bms.right.cell21 = R16(63) / 1000.0f;
          this->euc.bms.right.cell22 = R16(65) / 1000.0f;
          this->euc.bms.right.cell23 = R16(67) / 1000.0f;
          this->euc.bms.right.cell24 = R16(69) / 1000.0f;
          this->euc.bms.right.cell25 = R16(71) / 1000.0f;
          this->euc.bms.right.cell26 = R16(73) / 1000.0f;
          this->euc.bms.right.cell27 = R16(75) / 1000.0f;
          this->euc.bms.right.cell28 = R16(77) / 1000.0f;
          this->euc.bms.right.cell29 = R16(79) / 1000.0f;
          this->euc.bms.right.cell30 = R16(81) / 1000.0f;
        }
        break;
      case 0x07:
        if (size >= 70) {
          this->euc.bms.right.temp1 = R16(47) / 100.0f;
          this->euc.bms.right.temp2 = R16(49) / 100.0f;
          this->euc.bms.right.temp3 = R16(51) / 100.0f;
          this->euc.bms.right.temp4 = R16(53) / 100.0f;
          this->euc.bms.right.temp5 = R16(55) / 100.0f;
          this->euc.bms.right.temp6 = R16(57) / 100.0f;
          this->euc.bms.right.cell31 = R16(59) / 1000.0f;
          this->euc.bms.right.cell32 = R16(61) / 1000.0f;
          this->euc.bms.right.cell33 = R16(63) / 1000.0f;
          this->euc.bms.right.cell34 = R16(65) / 1000.0f;
          this->euc.bms.right.cell35 = R16(67) / 1000.0f;
          this->euc.bms.right.cell36 = R16(69) / 1000.0f;
          this->sensor_bms_right_temp_1_->publish_state(this->euc.bms.right.temp1);
          this->sensor_bms_right_temp_2_->publish_state(this->euc.bms.right.temp2);
          this->sensor_bms_right_temp_3_->publish_state(this->euc.bms.right.temp3);
          this->sensor_bms_right_temp_4_->publish_state(this->euc.bms.right.temp4);
          this->sensor_bms_right_temp_5_->publish_state(this->euc.bms.right.temp5);
          this->sensor_bms_right_temp_6_->publish_state(this->euc.bms.right.temp6);
        }
        break;
      case 0x08:
        if (size >= 66) {
          this->euc.headlight = data[47] == 0x01;
          this->euc.low_power_mode = data[60] == 0x01;
          this->euc.high_speed_mode = data[61] == 0x01;
          this->euc.cut_off_angle = data[62];
          this->euc.tho_ra = data[66];
          this->euc.charging_stop_voltage = R16(63) + 682;
          this->binary_sensor_headlight_->publish_state(this->euc.headlight);
          this->binary_sensor_high_speed_mode_->publish_state(this->euc.high_speed_mode);
          this->binary_sensor_low_power_mode_->publish_state(this->euc.low_power_mode);
          if (this->sensor_charging_stop_voltage_ != nullptr) {
            this->sensor_charging_stop_voltage_->publish_state(this->euc.charging_stop_voltage / 10.0f);
          }
          this->sensor_tho_ra_->publish_state(this->euc.tho_ra);
          if (this->max_charging_voltage_number_ != nullptr) {
            this->max_charging_voltage_number_->publish_state(this->euc.charging_stop_voltage / 10.0f);
          }
        }
        break;
    }
  }
#undef R16
#undef R16s
#undef R32m
}

std::vector<uint8_t> VeteranComponent::get_charge_packet(float voltage) {
  // База пакета «макс. напряжение заряда» (25 байт), как в 7.3/7.4 veteran-protocol.md
  std::vector<uint8_t> packet = {0x4C, 0x64, 0x41, 0x70, 0x1D, 0x01, 0x02, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x00};
  // Кодирование: byte[24] = (voltage - 145) * 10 (151.2→62, 147→20 по рабочим пакетам)
  float v = (voltage - 145.0f) * 10.0f;
  packet[24] = (uint8_t) (v < 0 ? 0 : (v > 255 ? 255 : (uint8_t) (v + 0.5f)));
  uint32_t crc = esp_crc32_le(0, packet.data(), 25);
  packet.push_back((crc >> 24) & 0xFF);
  packet.push_back((crc >> 16) & 0xFF);
  packet.push_back((crc >> 8) & 0xFF);
  packet.push_back(crc & 0xFF);
  return packet;
}

uint16_t VeteranComponent::unsignedShortFromBytesBE(const std::vector<uint8_t> &bytes, int offset) {
  if (bytes.empty() || bytes.size() < offset + 2) {
    ESP_LOGE("unsignedShortFromBytesBE", "bytes is empty or too short");
    return 0;
  }

  uint8_t high = bytes[offset], low = bytes[offset + 1];
  uint16_t v = high << 8 | low;
  return v;
}

int16_t VeteranComponent::shortFromBytesBE(const std::vector<uint8_t> &bytes, int offset) {
  if (bytes.empty() || bytes.size() < offset + 2) {
    ESP_LOGE("shortFromBytesBE", "bytes is empty or too short");
    return 0;
  }

  int8_t high = bytes[offset], low = bytes[offset + 1];
  int16_t v = high << 8 | low;
  return v;
}

uint32_t VeteranComponent::longFromBytesBE(const std::vector<uint8_t> &bytes, int offset) {
  if (bytes.empty() || bytes.size() < offset + 4) {
    ESP_LOGE("longFromBytesBE", "bytes is empty or too short");
    return 0;
  }
  return (((bytes[offset] & 0xFF) << 24) | ((bytes[offset + 1] & 0xFF) << 16) | ((bytes[offset + 2] & 0xFF) << 8) | (bytes[offset + 3] & 0xFF)) & 0xFFFFFFFFL;
}

uint32_t VeteranComponent::unsignedLongFromBytesMidLE(const std::vector<uint8_t> &bytes, int offset) {
  if (bytes.empty() || bytes.size() < offset + 4) {
    ESP_LOGE("unsignedLongFromBytesMidLE", "bytes is empty or too short");
    return 0;
  }

  return (((bytes[offset + 2] & 0xFF) << 24) | ((bytes[offset + 3] & 0xFF) << 16) | ((bytes[offset] & 0xFF) << 8) | (bytes[offset + 1] & 0xFF));
}

bool VeteranComponent::check_crc32(const uint8_t *data, size_t size) {
  if (size < 4) return false;
  size_t data_len = size - 4;
  uint32_t crc_calc = esp_crc32_le(0, data, data_len);
  uint32_t crc_recv = (uint32_t) (data[data_len] << 24) | (data[data_len + 1] << 16) | (data[data_len + 2] << 8) | data[data_len + 3];
  if (crc_recv == crc_calc) return true;
  ESP_LOGW("CRC", "(recv=%08lX calc=%08lX)", (unsigned long) crc_recv, (unsigned long) crc_calc);
  return false;
}

bool VeteranComponent::check_crc32(const std::vector<uint8_t> &bytes) {
  return check_crc32(bytes.data(), bytes.size());
}
}  // namespace veteran
}  // namespace esphome
