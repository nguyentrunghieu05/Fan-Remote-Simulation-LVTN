#include "test.h"
int g_failures = 0, g_checks = 0;

void suite_ring_buffer(void);
void suite_crc(void);
void suite_modbus(void);
void suite_pid_ramp(void);
void suite_fault(void);
void suite_timeconv(void);
void suite_cli(void);
void suite_datalog(void);
void suite_config_store(void);
void suite_w25qxx(void);
void suite_sht3x(void);
void suite_i2c_devices(void);
void suite_mcp2515(void);
void suite_telemetry(void);
void suite_app(void);

int main(void)
{
    suite_ring_buffer();
    suite_crc();
    suite_modbus();
    suite_pid_ramp();
    suite_fault();
    suite_timeconv();
    suite_cli();
    suite_datalog();
    suite_config_store();
    suite_w25qxx();
    suite_sht3x();
    suite_i2c_devices();
    suite_mcp2515();
    suite_telemetry();
    suite_app();
    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures ? 1 : 0;
}
