#include "test.h"
#include "app_fsm.h"
#include "fan_logic.h"

static void fsm_modes(void)
{
    app_fsm_t f; app_fsm_init(&f, APP_STATE_OFF);
    CHECK(app_fsm_handle(&f, APP_EV_BUTTON));      CHECK_EQ(f.state, APP_STATE_MANUAL);
    CHECK(app_fsm_handle(&f, APP_EV_BUTTON));      CHECK_EQ(f.state, APP_STATE_AUTO);
    CHECK(app_fsm_handle(&f, APP_EV_FAULT_RAISED)); CHECK_EQ(f.state, APP_STATE_FAULT);
    CHECK(!app_fsm_handle(&f, APP_EV_MODE_MANUAL)); CHECK_EQ(f.state, APP_STATE_FAULT);  /* remembered */
    CHECK(app_fsm_handle(&f, APP_EV_FAULT_CLEARED)); CHECK_EQ(f.state, APP_STATE_MANUAL);
    CHECK(!app_fsm_handle(&f, APP_EV_FAULT_CLEARED));
    CHECK(app_fsm_handle(&f, APP_EV_MODE_OFF));    CHECK_EQ(f.state, APP_STATE_OFF);
    app_fsm_init(&f, APP_STATE_FAULT);             CHECK_EQ(f.state, APP_STATE_OFF);
}

/* Closed loop with a crude thermal model: heat input + cooling ~ duty. */
static void auto_mode_converges(void)
{
    fan_config_t c; fan_config_defaults(&c);
    CHECK(!fan_config_sanitize(&c));
    fan_logic_t l; fan_logic_init(&l, &c);
    fan_inputs_t in = { .temp_c = 40.0f, .temp_valid = true, .rpm = 1000, .pot_percent = 0 };
    float duty = 0;
    for (int i = 0; i < 6000; i++) {                     /* 10 min at 100 ms */
        duty = fan_logic_step(&l, &c, APP_STATE_AUTO, &in, 0.1f);
        float heat = 0.05f, cool = duty * 0.0012f, ambient_loss = (in.temp_c - 25.0f) * 0.001f;
        in.temp_c += heat - cool - ambient_loss;
    }
    CHECK_NEAR(in.temp_c, 30.0f, 0.3f);
    CHECK(duty > 20.0f && duty < 100.0f);
}
static void manual_off_fault(void)
{
    fan_config_t c; fan_config_defaults(&c);
    c.use_pot = 0; c.manual_duty_c10 = 100;              /* 10 % < min 20 % */
    fan_logic_t l; fan_logic_init(&l, &c);
    fan_inputs_t in = { 25.0f, true, 0, 80.0f };
    float d = 0;
    for (int i = 0; i < 50; i++) d = fan_logic_step(&l, &c, APP_STATE_MANUAL, &in, 0.1f);
    CHECK_NEAR(d, 20.0f, 0.01f);                         /* raised to minimum */
    c.use_pot = 1;
    for (int i = 0; i < 50; i++) d = fan_logic_step(&l, &c, APP_STATE_MANUAL, &in, 0.1f);
    CHECK_NEAR(d, 80.0f, 0.01f);                         /* follows the knob */
    d = fan_logic_step(&l, &c, APP_STATE_OFF, &in, 0.1f);
    CHECK_NEAR(d, 78.0f, 0.01f);                         /* ramps down, no jump */
    for (int i = 0; i < 50; i++) d = fan_logic_step(&l, &c, APP_STATE_OFF, &in, 0.1f);
    CHECK_NEAR(d, 0.0f, 0.01f);
    for (int i = 0; i < 60; i++) d = fan_logic_step(&l, &c, APP_STATE_FAULT, &in, 0.1f);
    CHECK_NEAR(d, 100.0f, 0.01f);                        /* fail safe */
    in.temp_valid = false;
    d = fan_logic_step(&l, &c, APP_STATE_AUTO, &in, 0.1f);
    CHECK_NEAR(d, 100.0f, 0.01f);                        /* no sensor in auto -> safe */
}
static void config_sanitize(void)
{
    fan_config_t c; fan_config_defaults(&c);
    c.modbus_addr = 0; c.kp = -3; c.log_period_s = 0;
    CHECK(fan_config_sanitize(&c));
    CHECK_EQ(c.modbus_addr, 1); CHECK_NEAR(c.kp, 0, 1e-6); CHECK_EQ(c.log_period_s, 1);
    c.version = 99;
    CHECK(fan_config_sanitize(&c)); CHECK_EQ(c.version, 1); CHECK_EQ(c.setpoint_c100, 3000);
}
void suite_app(void) { RUN(fsm_modes); RUN(auto_mode_converges); RUN(manual_off_fault); RUN(config_sanitize); }
