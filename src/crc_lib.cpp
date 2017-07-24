#include <teraranger_hub/crc_lib.h>

namespace teraranger_hub
{
    uint8_t Crc::crc8(uint8_t *p, uint8_t len)
    {
        uint16_t i;
        uint16_t crc = 0x0;

        while (len--)
        {
            i = (crc ^ *p++) & 0xFF;
            crc = (crc_table[i] ^ (crc << 8)) & 0xFF;
        }
        return crc & 0xFF;
    }
} //namespace teraranger_hub
