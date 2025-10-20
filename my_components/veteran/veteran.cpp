#include "veteran.h"

namespace esphome {
namespace veteran{

void VeteranComponent::parse_ble_packet(const std::vector<uint8_t>& x)
{
    // ESP_LOGD("BLE", "(%zu) %s", x.size(), format_hex_pretty(x, ' ', false).c_str());

    this->ble_buffer_.insert(this->ble_buffer_.end(), x.begin(), x.end());

    constexpr uint8_t HEADER[] = {0xDC, 0x5A, 0x5C};
    constexpr size_t HEADER_SIZE = sizeof(HEADER);

    while (this->ble_buffer_.size() >= HEADER_SIZE)
    {
        // Пока не наткнулись на заголовок - удаляем начало
        if (!std::equal(HEADER, HEADER + HEADER_SIZE, this->ble_buffer_.begin())) {
            this->ble_buffer_.erase(this->ble_buffer_.begin());
            continue;
        }

        // Ждем третий байт (длину пакета)
        if (this->ble_buffer_.size() < HEADER_SIZE + 1) break;
        uint8_t length = ble_buffer_[3];
        
        // заголовок + длина пакета + байт длины + 1 байт для CRC
        size_t expected_length = HEADER_SIZE + length + 1;

        if (this->ble_buffer_.size() < expected_length) break; // ждём полный пакет
        
        std::vector<uint8_t> bytes(this->ble_buffer_.begin(), this->ble_buffer_.begin() + expected_length);
        if (check_crc32(bytes, length)) {
            // ESP_LOGD("EUC", "%s", format_hex_pretty(bytes, ' ', false).c_str());
            parse_packet(bytes);
        }

        this->ble_buffer_.erase(this->ble_buffer_.begin(), this->ble_buffer_.begin() + expected_length);
    }

    if (this->ble_buffer_.size() > 1024) this->ble_buffer_.clear();
}

void VeteranComponent::parse_packet(const std::vector<uint8_t>& bytes)
{
    this->euc.voltage = unsignedShortFromBytesBE(bytes, 4);
    this->euc.speed = shortFromBytesBE(bytes, 6);
    this->euc.mileage_current = unsignedLongFromBytesMidLE(bytes, 8) / 1000.0f;
    this->euc.mileage_total = unsignedLongFromBytesMidLE(bytes, 12) / 1000.0f;
    this->euc.phase_current = shortFromBytesBE(bytes, 16);
    this->euc.temperature_motor = unsignedShortFromBytesBE(bytes, 18) / 100.0f;
    this->euc.auto_off = unsignedShortFromBytesBE(bytes, 20);
    this->euc.charging = bytes[22] == 0x01;
    this->euc.speed_alert = unsignedShortFromBytesBE(bytes, 24);
    this->euc.speed_tiltback = unsignedShortFromBytesBE(bytes, 26);

    uint16_t fw = unsignedShortFromBytesBE(bytes, 28);
    this->euc.modelVersion = fw / 1000;
    
    snprintf(this->euc.firmware_version, 9, "%03d.%01d.%02d", this->euc.modelVersion, (fw / 100) % 10, fw % 100);

    this->euc.pedals_mode = unsignedShortFromBytesBE(bytes, 30) - 100;
    this->euc.pitch_angle = shortFromBytesBE(bytes, 32); // наклон вперед/назад | если около 6248 - стоит на родной подножке
    this->euc.pwm = unsignedShortFromBytesBE(bytes, 34);

    this->sensor_voltage_->publish_state(this->euc.voltage / 100.0f);
    // this->sensor_speed_->publish_state(this->euc.speed);
    this->sensor_mileage_current_->publish_state(this->euc.mileage_current);
    this->sensor_mileage_total_->publish_state(this->euc.mileage_total);
    // this->sensor_phase_current_->publish_state(this->euc.phase_current);
    this->sensor_temperature_motor_->publish_state(this->euc.temperature_motor);
    this->sensor_auto_off_->publish_state(this->euc.auto_off / 60.0f);
    this->binary_sensor_charging_->publish_state(this->euc.charging);
    // this->sensor_speed_alert_->publish_state(this->euc.speed_alert);
    // this->sensor_speed_tiltback_->publish_state(this->euc.speed_tiltback);
    this->text_sensor_firmware_version_->publish_state(this->euc.firmware_version);

    // Calculated sensors
    this->sensor_battery_percentage_->publish_state(this->euc.battery_percentage());

    if (this->euc.modelVersion >= 5) {
        // 5 - Veteran Lynx
        // 6 - Veteran Sherman L

        if (bytes[46] == 0x00 || bytes[46] == 0x04) {
            // 0 - livedata
            // Ток: отрицательное значение - заряд, положительное - разряд
            this->euc.temperature_controller = unsignedShortFromBytesBE(bytes, 59) / 100.0f; // Температура, как на экране
            this->euc.bms.left.current = shortFromBytesBE(bytes, 69) / -100.0f; // Ток левой батареи
            this->euc.bms.right.current = shortFromBytesBE(bytes, 71) / -100.0f; // Ток правой батареи

            this->sensor_bms_left_current_->publish_state(this->euc.bms.left.current);
            this->sensor_bms_right_current_->publish_state(this->euc.bms.right.current);
            this->sensor_temperature_controller_->publish_state(this->euc.temperature_controller);
        } else if (bytes[46] == 0x01) {
            this->euc.bms.left.cell01 = unsignedShortFromBytesBE(bytes, 53) / 1000.0f;
            this->euc.bms.left.cell02 = unsignedShortFromBytesBE(bytes, 55) / 1000.0f;
            this->euc.bms.left.cell03 = unsignedShortFromBytesBE(bytes, 57) / 1000.0f;
            this->euc.bms.left.cell04 = unsignedShortFromBytesBE(bytes, 59) / 1000.0f;
            this->euc.bms.left.cell05 = unsignedShortFromBytesBE(bytes, 61) / 1000.0f;
            this->euc.bms.left.cell06 = unsignedShortFromBytesBE(bytes, 63) / 1000.0f;
            this->euc.bms.left.cell07 = unsignedShortFromBytesBE(bytes, 65) / 1000.0f;
            this->euc.bms.left.cell08 = unsignedShortFromBytesBE(bytes, 67) / 1000.0f;
            this->euc.bms.left.cell09 = unsignedShortFromBytesBE(bytes, 69) / 1000.0f;
            this->euc.bms.left.cell10 = unsignedShortFromBytesBE(bytes, 71) / 1000.0f;
            this->euc.bms.left.cell11 = unsignedShortFromBytesBE(bytes, 73) / 1000.0f;
            this->euc.bms.left.cell12 = unsignedShortFromBytesBE(bytes, 75) / 1000.0f;
            this->euc.bms.left.cell13 = unsignedShortFromBytesBE(bytes, 77) / 1000.0f;
            this->euc.bms.left.cell14 = unsignedShortFromBytesBE(bytes, 79) / 1000.0f;
            this->euc.bms.left.cell15 = unsignedShortFromBytesBE(bytes, 81) / 1000.0f;

            this->sensor_bms_left_cell_01_->publish_state(this->euc.bms.left.cell01);
            this->sensor_bms_left_cell_02_->publish_state(this->euc.bms.left.cell02);
            this->sensor_bms_left_cell_03_->publish_state(this->euc.bms.left.cell03);
            this->sensor_bms_left_cell_04_->publish_state(this->euc.bms.left.cell04);
            this->sensor_bms_left_cell_05_->publish_state(this->euc.bms.left.cell05);
            this->sensor_bms_left_cell_06_->publish_state(this->euc.bms.left.cell06);
            this->sensor_bms_left_cell_07_->publish_state(this->euc.bms.left.cell07);
            this->sensor_bms_left_cell_08_->publish_state(this->euc.bms.left.cell08);
            this->sensor_bms_left_cell_09_->publish_state(this->euc.bms.left.cell09);
            this->sensor_bms_left_cell_10_->publish_state(this->euc.bms.left.cell10);
            this->sensor_bms_left_cell_11_->publish_state(this->euc.bms.left.cell11);
            this->sensor_bms_left_cell_12_->publish_state(this->euc.bms.left.cell12);
            this->sensor_bms_left_cell_13_->publish_state(this->euc.bms.left.cell13);
            this->sensor_bms_left_cell_14_->publish_state(this->euc.bms.left.cell14);
            this->sensor_bms_left_cell_15_->publish_state(this->euc.bms.left.cell15);
        } else if (bytes[46] == 0x02) {
            this->euc.bms.left.cell16 = unsignedShortFromBytesBE(bytes, 53) / 1000.0f;
            this->euc.bms.left.cell17 = unsignedShortFromBytesBE(bytes, 55) / 1000.0f;
            this->euc.bms.left.cell18 = unsignedShortFromBytesBE(bytes, 57) / 1000.0f;
            this->euc.bms.left.cell19 = unsignedShortFromBytesBE(bytes, 59) / 1000.0f;
            this->euc.bms.left.cell20 = unsignedShortFromBytesBE(bytes, 61) / 1000.0f;
            this->euc.bms.left.cell21 = unsignedShortFromBytesBE(bytes, 63) / 1000.0f;
            this->euc.bms.left.cell22 = unsignedShortFromBytesBE(bytes, 65) / 1000.0f;
            this->euc.bms.left.cell23 = unsignedShortFromBytesBE(bytes, 67) / 1000.0f;
            this->euc.bms.left.cell24 = unsignedShortFromBytesBE(bytes, 69) / 1000.0f;
            this->euc.bms.left.cell25 = unsignedShortFromBytesBE(bytes, 71) / 1000.0f;
            this->euc.bms.left.cell26 = unsignedShortFromBytesBE(bytes, 73) / 1000.0f;
            this->euc.bms.left.cell27 = unsignedShortFromBytesBE(bytes, 75) / 1000.0f;
            this->euc.bms.left.cell28 = unsignedShortFromBytesBE(bytes, 77) / 1000.0f;
            this->euc.bms.left.cell29 = unsignedShortFromBytesBE(bytes, 79) / 1000.0f;
            this->euc.bms.left.cell30 = unsignedShortFromBytesBE(bytes, 81) / 1000.0f;

            this->sensor_bms_left_cell_16_->publish_state(this->euc.bms.left.cell16);
            this->sensor_bms_left_cell_17_->publish_state(this->euc.bms.left.cell17);
            this->sensor_bms_left_cell_18_->publish_state(this->euc.bms.left.cell18);
            this->sensor_bms_left_cell_19_->publish_state(this->euc.bms.left.cell19);
            this->sensor_bms_left_cell_20_->publish_state(this->euc.bms.left.cell20);
            this->sensor_bms_left_cell_21_->publish_state(this->euc.bms.left.cell21);
            this->sensor_bms_left_cell_22_->publish_state(this->euc.bms.left.cell22);
            this->sensor_bms_left_cell_23_->publish_state(this->euc.bms.left.cell23);
            this->sensor_bms_left_cell_24_->publish_state(this->euc.bms.left.cell24);
            this->sensor_bms_left_cell_25_->publish_state(this->euc.bms.left.cell25);
            this->sensor_bms_left_cell_26_->publish_state(this->euc.bms.left.cell26);
            this->sensor_bms_left_cell_27_->publish_state(this->euc.bms.left.cell27);
            this->sensor_bms_left_cell_28_->publish_state(this->euc.bms.left.cell28);
            this->sensor_bms_left_cell_29_->publish_state(this->euc.bms.left.cell29);
            this->sensor_bms_left_cell_30_->publish_state(this->euc.bms.left.cell30);
        } else if (bytes[46] == 0x03) {
            this->euc.bms.left.temp1  = unsignedShortFromBytesBE(bytes, 47) / 100.0f;
            this->euc.bms.left.temp2  = unsignedShortFromBytesBE(bytes, 49) / 100.0f;
            this->euc.bms.left.temp3  = unsignedShortFromBytesBE(bytes, 51) / 100.0f;
            this->euc.bms.left.temp4  = unsignedShortFromBytesBE(bytes, 53) / 100.0f;
            this->euc.bms.left.temp5  = unsignedShortFromBytesBE(bytes, 55) / 100.0f;
            this->euc.bms.left.temp6  = unsignedShortFromBytesBE(bytes, 57) / 100.0f;
            
            this->euc.bms.left.cell31 = unsignedShortFromBytesBE(bytes, 59) / 1000.0f;
            this->euc.bms.left.cell32 = unsignedShortFromBytesBE(bytes, 61) / 1000.0f;
            this->euc.bms.left.cell33 = unsignedShortFromBytesBE(bytes, 63) / 1000.0f;
            this->euc.bms.left.cell34 = unsignedShortFromBytesBE(bytes, 65) / 1000.0f;
            this->euc.bms.left.cell35 = unsignedShortFromBytesBE(bytes, 67) / 1000.0f;
            this->euc.bms.left.cell36 = unsignedShortFromBytesBE(bytes, 69) / 1000.0f;

            this->sensor_bms_left_temp_1_->publish_state(this->euc.bms.left.temp1);
            this->sensor_bms_left_temp_2_->publish_state(this->euc.bms.left.temp2);
            this->sensor_bms_left_temp_3_->publish_state(this->euc.bms.left.temp3);
            this->sensor_bms_left_temp_4_->publish_state(this->euc.bms.left.temp4);
            this->sensor_bms_left_temp_5_->publish_state(this->euc.bms.left.temp5);
            this->sensor_bms_left_temp_6_->publish_state(this->euc.bms.left.temp6);
            
            this->sensor_bms_left_cell_31_->publish_state(this->euc.bms.left.cell31);
            this->sensor_bms_left_cell_32_->publish_state(this->euc.bms.left.cell32);
            this->sensor_bms_left_cell_33_->publish_state(this->euc.bms.left.cell33);
            this->sensor_bms_left_cell_34_->publish_state(this->euc.bms.left.cell34);
            this->sensor_bms_left_cell_35_->publish_state(this->euc.bms.left.cell35);
            this->sensor_bms_left_cell_36_->publish_state(this->euc.bms.left.cell36);
        } else if (bytes[46] == 0x05) {
            this->euc.bms.right.cell01 = unsignedShortFromBytesBE(bytes, 53) / 1000.0f;
            this->euc.bms.right.cell02 = unsignedShortFromBytesBE(bytes, 55) / 1000.0f;
            this->euc.bms.right.cell03 = unsignedShortFromBytesBE(bytes, 57) / 1000.0f;
            this->euc.bms.right.cell04 = unsignedShortFromBytesBE(bytes, 59) / 1000.0f;
            this->euc.bms.right.cell05 = unsignedShortFromBytesBE(bytes, 61) / 1000.0f;
            this->euc.bms.right.cell06 = unsignedShortFromBytesBE(bytes, 63) / 1000.0f;
            this->euc.bms.right.cell07 = unsignedShortFromBytesBE(bytes, 65) / 1000.0f;
            this->euc.bms.right.cell08 = unsignedShortFromBytesBE(bytes, 67) / 1000.0f;
            this->euc.bms.right.cell09 = unsignedShortFromBytesBE(bytes, 69) / 1000.0f;
            this->euc.bms.right.cell10 = unsignedShortFromBytesBE(bytes, 71) / 1000.0f;
            this->euc.bms.right.cell11 = unsignedShortFromBytesBE(bytes, 73) / 1000.0f;
            this->euc.bms.right.cell12 = unsignedShortFromBytesBE(bytes, 75) / 1000.0f;
            this->euc.bms.right.cell13 = unsignedShortFromBytesBE(bytes, 77) / 1000.0f;
            this->euc.bms.right.cell14 = unsignedShortFromBytesBE(bytes, 79) / 1000.0f;
            this->euc.bms.right.cell15 = unsignedShortFromBytesBE(bytes, 81) / 1000.0f;

            this->sensor_bms_right_cell_01_->publish_state(this->euc.bms.right.cell01);
            this->sensor_bms_right_cell_02_->publish_state(this->euc.bms.right.cell02);
            this->sensor_bms_right_cell_03_->publish_state(this->euc.bms.right.cell03);
            this->sensor_bms_right_cell_04_->publish_state(this->euc.bms.right.cell04);
            this->sensor_bms_right_cell_05_->publish_state(this->euc.bms.right.cell05);
            this->sensor_bms_right_cell_06_->publish_state(this->euc.bms.right.cell06);
            this->sensor_bms_right_cell_07_->publish_state(this->euc.bms.right.cell07);
            this->sensor_bms_right_cell_08_->publish_state(this->euc.bms.right.cell08);
            this->sensor_bms_right_cell_09_->publish_state(this->euc.bms.right.cell09);
            this->sensor_bms_right_cell_10_->publish_state(this->euc.bms.right.cell10);
            this->sensor_bms_right_cell_11_->publish_state(this->euc.bms.right.cell11);
            this->sensor_bms_right_cell_12_->publish_state(this->euc.bms.right.cell12);
            this->sensor_bms_right_cell_13_->publish_state(this->euc.bms.right.cell13);
            this->sensor_bms_right_cell_14_->publish_state(this->euc.bms.right.cell14);
            this->sensor_bms_right_cell_15_->publish_state(this->euc.bms.right.cell15);
        } else if (bytes[46] == 0x06) {
            this->euc.bms.right.cell16 = unsignedShortFromBytesBE(bytes, 53) / 1000.0f;
            this->euc.bms.right.cell17 = unsignedShortFromBytesBE(bytes, 55) / 1000.0f;
            this->euc.bms.right.cell18 = unsignedShortFromBytesBE(bytes, 57) / 1000.0f;
            this->euc.bms.right.cell19 = unsignedShortFromBytesBE(bytes, 59) / 1000.0f;
            this->euc.bms.right.cell20 = unsignedShortFromBytesBE(bytes, 61) / 1000.0f;
            this->euc.bms.right.cell21 = unsignedShortFromBytesBE(bytes, 63) / 1000.0f;
            this->euc.bms.right.cell22 = unsignedShortFromBytesBE(bytes, 65) / 1000.0f;
            this->euc.bms.right.cell23 = unsignedShortFromBytesBE(bytes, 67) / 1000.0f;
            this->euc.bms.right.cell24 = unsignedShortFromBytesBE(bytes, 69) / 1000.0f;
            this->euc.bms.right.cell25 = unsignedShortFromBytesBE(bytes, 71) / 1000.0f;
            this->euc.bms.right.cell26 = unsignedShortFromBytesBE(bytes, 73) / 1000.0f;
            this->euc.bms.right.cell27 = unsignedShortFromBytesBE(bytes, 75) / 1000.0f;
            this->euc.bms.right.cell28 = unsignedShortFromBytesBE(bytes, 77) / 1000.0f;
            this->euc.bms.right.cell29 = unsignedShortFromBytesBE(bytes, 79) / 1000.0f;
            this->euc.bms.right.cell30 = unsignedShortFromBytesBE(bytes, 81) / 1000.0f;

            this->sensor_bms_right_cell_16_->publish_state(this->euc.bms.right.cell16);
            this->sensor_bms_right_cell_17_->publish_state(this->euc.bms.right.cell17);
            this->sensor_bms_right_cell_18_->publish_state(this->euc.bms.right.cell18);
            this->sensor_bms_right_cell_19_->publish_state(this->euc.bms.right.cell19);
            this->sensor_bms_right_cell_20_->publish_state(this->euc.bms.right.cell20);
            this->sensor_bms_right_cell_21_->publish_state(this->euc.bms.right.cell21);
            this->sensor_bms_right_cell_22_->publish_state(this->euc.bms.right.cell22);
            this->sensor_bms_right_cell_23_->publish_state(this->euc.bms.right.cell23);
            this->sensor_bms_right_cell_24_->publish_state(this->euc.bms.right.cell24);
            this->sensor_bms_right_cell_25_->publish_state(this->euc.bms.right.cell25);
            this->sensor_bms_right_cell_26_->publish_state(this->euc.bms.right.cell26);
            this->sensor_bms_right_cell_27_->publish_state(this->euc.bms.right.cell27);
            this->sensor_bms_right_cell_28_->publish_state(this->euc.bms.right.cell28);
            this->sensor_bms_right_cell_29_->publish_state(this->euc.bms.right.cell29);
            this->sensor_bms_right_cell_30_->publish_state(this->euc.bms.right.cell30);
        } else if (bytes[46] == 0x07) {
            this->euc.bms.right.temp1  = unsignedShortFromBytesBE(bytes, 47) / 100.0f;
            this->euc.bms.right.temp2  = unsignedShortFromBytesBE(bytes, 49) / 100.0f;
            this->euc.bms.right.temp3  = unsignedShortFromBytesBE(bytes, 51) / 100.0f;
            this->euc.bms.right.temp4  = unsignedShortFromBytesBE(bytes, 53) / 100.0f;
            this->euc.bms.right.temp5  = unsignedShortFromBytesBE(bytes, 55) / 100.0f;
            this->euc.bms.right.temp6  = unsignedShortFromBytesBE(bytes, 57) / 100.0f;

            this->euc.bms.right.cell31 = unsignedShortFromBytesBE(bytes, 59) / 1000.0f;
            this->euc.bms.right.cell32 = unsignedShortFromBytesBE(bytes, 61) / 1000.0f;
            this->euc.bms.right.cell33 = unsignedShortFromBytesBE(bytes, 63) / 1000.0f;
            this->euc.bms.right.cell34 = unsignedShortFromBytesBE(bytes, 65) / 1000.0f;
            this->euc.bms.right.cell35 = unsignedShortFromBytesBE(bytes, 67) / 1000.0f;
            this->euc.bms.right.cell36 = unsignedShortFromBytesBE(bytes, 69) / 1000.0f;

            this->sensor_bms_right_temp_1_->publish_state(this->euc.bms.right.temp1);
            this->sensor_bms_right_temp_2_->publish_state(this->euc.bms.right.temp2);
            this->sensor_bms_right_temp_3_->publish_state(this->euc.bms.right.temp3);
            this->sensor_bms_right_temp_4_->publish_state(this->euc.bms.right.temp4);
            this->sensor_bms_right_temp_5_->publish_state(this->euc.bms.right.temp5);
            this->sensor_bms_right_temp_6_->publish_state(this->euc.bms.right.temp6);
            
            this->sensor_bms_right_cell_31_->publish_state(this->euc.bms.right.cell31);
            this->sensor_bms_right_cell_32_->publish_state(this->euc.bms.right.cell32);
            this->sensor_bms_right_cell_33_->publish_state(this->euc.bms.right.cell33);
            this->sensor_bms_right_cell_34_->publish_state(this->euc.bms.right.cell34);
            this->sensor_bms_right_cell_35_->publish_state(this->euc.bms.right.cell35);
            this->sensor_bms_right_cell_36_->publish_state(this->euc.bms.right.cell36);
        } else if (bytes[46] == 0x08) {
            // this->euc.gyro_level = unsignedShortFromBytesBE(bytes, 51);
            // this->euc.brightness = unsignedShortFromBytesBE(bytes, 55);
            this->euc.low_power_mode = bytes[60] == 0x01;
            this->euc.high_speed_mode = bytes[61] == 0x01;
            this->euc.cut_off_angle = bytes[62];
            this->euc.tho_ra = bytes[66];
            // this->euc.cra_lu = bytes[??];
            this->euc.charging_stop_voltage = unsignedShortFromBytesBE(bytes, 63) + 682;

            this->binary_sensor_low_power_mode_->publish_state(this->euc.low_power_mode);
            this->binary_sensor_high_speed_mode_->publish_state(this->euc.high_speed_mode);
            // this->sensor_cut_off_angle_->publish_state(this->euc.cut_off_angle);
            this->sensor_tho_ra_->publish_state(this->euc.tho_ra);
            // this->sensor_cra_lu_->publish_state(this->euc.cra_lu);
            this->sensor_charging_stop_voltage_->publish_state(this->euc.charging_stop_voltage / 10.0f);
        }
    }
}

void VeteranComponent::on_ble_disconnected()
{
    this->binary_sensor_charging_->publish_state(false);
    this->sensor_auto_off_->publish_state(NAN);
    this->sensor_bms_left_current_->publish_state(NAN);
    this->sensor_bms_right_current_->publish_state(NAN);
    this->sensor_temperature_controller_->publish_state(NAN);
    this->sensor_temperature_motor_->publish_state(NAN);
    this->sensor_voltage_->publish_state(NAN);
    this->sensor_bms_left_cell_01_->publish_state(NAN);
    this->sensor_bms_left_cell_02_->publish_state(NAN);
    this->sensor_bms_left_cell_03_->publish_state(NAN);
    this->sensor_bms_left_cell_04_->publish_state(NAN);
    this->sensor_bms_left_cell_05_->publish_state(NAN);
    this->sensor_bms_left_cell_06_->publish_state(NAN);
    this->sensor_bms_left_cell_07_->publish_state(NAN);
    this->sensor_bms_left_cell_08_->publish_state(NAN);
    this->sensor_bms_left_cell_09_->publish_state(NAN);
    this->sensor_bms_left_cell_10_->publish_state(NAN);
    this->sensor_bms_left_cell_11_->publish_state(NAN);
    this->sensor_bms_left_cell_12_->publish_state(NAN);
    this->sensor_bms_left_cell_13_->publish_state(NAN);
    this->sensor_bms_left_cell_14_->publish_state(NAN);
    this->sensor_bms_left_cell_15_->publish_state(NAN);
    this->sensor_bms_left_cell_16_->publish_state(NAN);
    this->sensor_bms_left_cell_17_->publish_state(NAN);
    this->sensor_bms_left_cell_18_->publish_state(NAN);
    this->sensor_bms_left_cell_19_->publish_state(NAN);
    this->sensor_bms_left_cell_20_->publish_state(NAN);
    this->sensor_bms_left_cell_21_->publish_state(NAN);
    this->sensor_bms_left_cell_22_->publish_state(NAN);
    this->sensor_bms_left_cell_23_->publish_state(NAN);
    this->sensor_bms_left_cell_24_->publish_state(NAN);
    this->sensor_bms_left_cell_25_->publish_state(NAN);
    this->sensor_bms_left_cell_26_->publish_state(NAN);
    this->sensor_bms_left_cell_27_->publish_state(NAN);
    this->sensor_bms_left_cell_28_->publish_state(NAN);
    this->sensor_bms_left_cell_29_->publish_state(NAN);
    this->sensor_bms_left_cell_30_->publish_state(NAN);
    this->sensor_bms_left_cell_31_->publish_state(NAN);
    this->sensor_bms_left_cell_32_->publish_state(NAN);
    this->sensor_bms_left_cell_33_->publish_state(NAN);
    this->sensor_bms_left_cell_34_->publish_state(NAN);
    this->sensor_bms_left_cell_35_->publish_state(NAN);
    this->sensor_bms_left_cell_36_->publish_state(NAN);
    this->sensor_bms_left_temp_1_->publish_state(NAN);
    this->sensor_bms_left_temp_2_->publish_state(NAN);
    this->sensor_bms_left_temp_3_->publish_state(NAN);
    this->sensor_bms_left_temp_4_->publish_state(NAN);
    this->sensor_bms_left_temp_5_->publish_state(NAN);
    this->sensor_bms_left_temp_6_->publish_state(NAN);
    this->sensor_bms_right_cell_01_->publish_state(NAN);
    this->sensor_bms_right_cell_02_->publish_state(NAN);
    this->sensor_bms_right_cell_03_->publish_state(NAN);
    this->sensor_bms_right_cell_04_->publish_state(NAN);
    this->sensor_bms_right_cell_05_->publish_state(NAN);
    this->sensor_bms_right_cell_06_->publish_state(NAN);
    this->sensor_bms_right_cell_07_->publish_state(NAN);
    this->sensor_bms_right_cell_08_->publish_state(NAN);
    this->sensor_bms_right_cell_09_->publish_state(NAN);
    this->sensor_bms_right_cell_10_->publish_state(NAN);
    this->sensor_bms_right_cell_11_->publish_state(NAN);
    this->sensor_bms_right_cell_12_->publish_state(NAN);
    this->sensor_bms_right_cell_13_->publish_state(NAN);
    this->sensor_bms_right_cell_14_->publish_state(NAN);
    this->sensor_bms_right_cell_15_->publish_state(NAN);
    this->sensor_bms_right_cell_16_->publish_state(NAN);
    this->sensor_bms_right_cell_17_->publish_state(NAN);
    this->sensor_bms_right_cell_18_->publish_state(NAN);
    this->sensor_bms_right_cell_19_->publish_state(NAN);
    this->sensor_bms_right_cell_20_->publish_state(NAN);
    this->sensor_bms_right_cell_21_->publish_state(NAN);
    this->sensor_bms_right_cell_22_->publish_state(NAN);
    this->sensor_bms_right_cell_23_->publish_state(NAN);
    this->sensor_bms_right_cell_24_->publish_state(NAN);
    this->sensor_bms_right_cell_25_->publish_state(NAN);
    this->sensor_bms_right_cell_26_->publish_state(NAN);
    this->sensor_bms_right_cell_27_->publish_state(NAN);
    this->sensor_bms_right_cell_28_->publish_state(NAN);
    this->sensor_bms_right_cell_29_->publish_state(NAN);
    this->sensor_bms_right_cell_30_->publish_state(NAN);
    this->sensor_bms_right_cell_31_->publish_state(NAN);
    this->sensor_bms_right_cell_32_->publish_state(NAN);
    this->sensor_bms_right_cell_33_->publish_state(NAN);
    this->sensor_bms_right_cell_34_->publish_state(NAN);
    this->sensor_bms_right_cell_35_->publish_state(NAN);
    this->sensor_bms_right_cell_36_->publish_state(NAN);
    this->sensor_bms_right_temp_1_->publish_state(NAN);
    this->sensor_bms_right_temp_2_->publish_state(NAN);
    this->sensor_bms_right_temp_3_->publish_state(NAN);
    this->sensor_bms_right_temp_4_->publish_state(NAN);
    this->sensor_bms_right_temp_5_->publish_state(NAN);
    this->sensor_bms_right_temp_6_->publish_state(NAN);
}
} // namespace veteran
} // namespace esphome