/**
 * @file    modbus_server.h
 * @brief   Modbus RTU slave on RS485 for a PLC / HMI / SCADA.
 *
 * Input registers (FC 04, read only)
 *   0  temperature          0.01 °C (signed, 0x8000 = no sensor)
 *   1  humidity             0.01 %RH
 *   2  fan speed            rpm
 *   3  output duty          0.1 %
 *   4  state                0 OFF, 1 MANUAL, 2 AUTO, 3 FAULT
 *   5  fault bits           b0 stall, b1 sensor, b2 over-temp, b3 storage
 *   6  knob position        0.1 %
 *   7  uptime               s, high word
 *   8  uptime               s, low word
 *   9  log records          high word
 *  10  log records          low word
 *  11  firmware version     0x0100 = 1.0
 *
 * Holding registers (FC 03 read, FC 06 / FC 16 write)
 *   0  mode command         0 OFF, 1 MANUAL, 2 AUTO (reads current mode)
 *   1  manual duty          0.1 %      0..1000
 *   2  temperature setpoint 0.01 °C    0..8000
 *   3  Kp                   x100
 *   4  Ki                   x100
 *   5  Kd                   x100
 *   6  minimum duty         0.1 %
 *   7  fault duty           0.1 %
 *   8  over-temp limit      0.01 °C
 *   9  ramp rate            0.1 %/s
 *  10  log period           s
 *  11  telemetry period     s
 *  12  Modbus address       1..247 (applied after "save" + reboot)
 *  13  use knob in MANUAL   0 / 1
 *  14  stall rpm            0 = stall detection off
 *  20  action               1 save settings, 2 clear faults, 3 reboot
 */
#ifndef MODBUS_SERVER_H
#define MODBUS_SERVER_H

#include <stdint.h>

#define FW_VERSION_REG  0x0100U

void modbus_server_init(void);
void modbus_server_task(void *arg);
void modbus_server_stats(uint32_t *frames, uint32_t *crc_err, uint32_t *exceptions);

#endif /* MODBUS_SERVER_H */
