#ifndef C_BOARD_PROTOCOL_CRC_H
#define C_BOARD_PROTOCOL_CRC_H

#include <stdint.h>

namespace ProtocolCRC {

uint8_t GetCRC8CheckSum(const uint8_t *data, uint32_t length, uint8_t init = 0xFF);
uint16_t GetCRC16CheckSum(const uint8_t *data, uint32_t length, uint16_t init = 0xFFFF);

bool VerifyCRC8CheckSum(const uint8_t *data, uint32_t length, uint8_t init = 0xFF);
bool VerifyCRC16CheckSum(const uint8_t *data, uint32_t length, uint16_t init = 0xFFFF);

void AppendCRC8CheckSum(uint8_t *data, uint32_t length, uint8_t init = 0xFF);
void AppendCRC16CheckSum(uint8_t *data, uint32_t length, uint16_t init = 0xFFFF);
void AppendCRC8CRC16CheckSum(uint8_t *data,
                             uint16_t total_length,
                             uint8_t crc8_init = 0xFF,
                             uint16_t crc16_init = 0xFFFF);

}  // namespace ProtocolCRC

#endif  // C_BOARD_PROTOCOL_CRC_H
