/* Native smoke test of the simulator core: gcc build, prints a short run. */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
void sim_init(uint32_t); void sim_run(int); float sim_get(int); void sim_set(int, float);
const char *sim_console_out(void); const char *sim_air_out(void); void sim_outputs_clear(void);
char *sim_cli_buf(void); void sim_cli(void); uint8_t *sim_mb_req_buf(void); uint8_t *sim_mb_resp_buf(void);
int sim_modbus(int); void sim_reset(void);
static void cli(const char *s) { strcpy(sim_cli_buf(), s); sim_cli(); }
int main(void)
{
    sim_init(1790000000u);
    sim_set(1, 80);                         /* heavy heat load */
    for (int s = 0; s <= 300; s += 30) {
        printf("t=%3ds T=%.2f meas=%.2f duty=%.1f rpm=%.0f state=%.0f faults=%.0f log=%.0f sent=%.0f ack=%.0f\n",
               s, sim_get(0), sim_get(1), sim_get(3), sim_get(4), sim_get(5), sim_get(6), sim_get(8), sim_get(14), sim_get(15));
        sim_run(300);
    }
    cli("status"); cli("i2c"); cli("flash"); cli("log dump 3"); cli("stats");
    /* Modbus FC04 read 6 input regs from slave 1 */
    uint8_t *q = sim_mb_req_buf();
    uint8_t f[] = { 0x01, 0x04, 0x00, 0x00, 0x00, 0x06 };
    memcpy(q, f, 6);
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 6; i++) { crc ^= q[i]; for (int b = 0; b < 8; b++) crc = (crc & 1) ? (crc >> 1) ^ 0xA001 : crc >> 1; }
    q[6] = crc & 0xFF; q[7] = crc >> 8;
    int n = sim_modbus(8);
    printf("modbus reply %d bytes:", n);
    for (int i = 0; i < n; i++) printf(" %02X", sim_mb_resp_buf()[i]);
    printf("\n");
    sim_set(3, 1);  sim_run(100);            /* jam the fan */
    sim_set(4, 1);  sim_run(100);            /* unplug sensor */
    printf("after faults: state=%.0f faults=%.0f duty=%.1f relay=%.0f\n", sim_get(5), sim_get(6), sim_get(3), sim_get(11));
    sim_reset(); sim_run(50);
    printf("after reset: log=%.0f\n", sim_get(8));
    fputs(sim_console_out(), stdout);
    fputs("---- air ----\n", stdout);
    const char *a = sim_air_out(); printf("%.600s\n", a);
    return 0;
}
