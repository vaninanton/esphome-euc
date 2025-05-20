#include "esphome.h"
void ESP_LOGD_HEX(std::vector<uint8_t> bytes, uint8_t separator)
{
    std::string res;
    size_t len = bytes.size();
    char buf[5];
    for (size_t i = 0; i < len; i++)
    {
        if (i > 0)
        {
            res += separator;
        }
        sprintf(buf, "%02X", bytes[i]);
        res += buf;
    }
    ESP_LOGD("HEX", "%s", res.c_str());
}
