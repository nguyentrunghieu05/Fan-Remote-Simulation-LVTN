/**
 * @file    crc.h
 * @brief   Checksums used in the project.
 *
 *  - CRC-16/MODBUS : Modbus RTU frames (poly 0xA001 reflected, init 0xFFFF)
 *  - CRC-8/SENSIRION: SHT3x measurement words (poly 0x31, init 0xFF)
 *  - CRC-32 (IEEE)  : stored config and data-log records
 */
#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stddef.h>

uint16_t crc16_modbus(const uint8_t *data, size_t len);
uint8_t  crc8_sensirion(const uint8_t *data, size_t len);
uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len);
uint32_t crc32(const uint8_t *data, size_t len);

#endif /* CRC_H */
