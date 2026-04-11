#ifndef POWERPACK_PARAM_TABLE_H_
#define POWERPACK_PARAM_TABLE_H_

#include <stdint.h>

// Production configuration parameters from "OpenPPG V1.11 Config Shipping"
// Each entry: { param_name, integer_value }
// String and array params handled separately.

struct ParamEntry {
    const char* name;
    int64_t value;
};

// Production config - derived from the JSON config file.
// Array/curve params are excluded here (handled separately if needed).
static const ParamEntry PRODUCTION_CONFIG[] = {
    // Board params
    {"node_id",                     32},
    {"can_baudrate",                0},       // 0 = 1000K
    {"uart_baudrate",               0},       // 0 = 115200
    {"vbus_upper_limit",            108},
    {"vbus_lower_limit",            48},
    {"over_voltage_threshold",      105},
    {"overt_voltage_tolerance",     6},
    {"overt_voltage_tolerance2",    8},
    {"dcbus_lpf_hz",                500},
    {"led_color",                   1},       // 1 = Green
    {"standby_led_type",            1},       // 1 = Always On
    {"rs485_led_port",              0},       // 0 = Led

    // Motor params
    {"motor_pole_pairs",            31},
    {"motor_rs",                    147},
    {"motor_ld",                    122},
    {"motor_lq",                    122},
    {"ls_coef",                     100},
    {"motor_kv",                    340},
    {"motor_max_current",           2200},
    {"carrier_freq_khz",            16},
    {"current_loop_coef",           25},

    // Control params
    {"ctrl_input_type",             0},       // 0 = Pwm+CommPwm
    {"throttle_recover_check",      1},       // 1 = Enable
    {"direction",                   1},       // 1 = Inversion
    {"idling_speed_rpm",            60},
    {"motor_max_rpm",               2500},
    {"stop_type",                   0},       // 0 = Free Stop
    {"min_startup_speed_enable",    0},       // Disable
    {"min_startup_speed",           200},

    // Speed params
    {"idling_acc_krpmps",           300},
    {"max_acc_krpmps",              300},
    {"max_dec_krpmps",              300},
    {"max_accel_current",           2100},
    {"max_decel_current",           -15},
    {"acc_type",                    0},       // 0 = Normal Mode
    {"idling_acc_speed_rpm",        1},
    {"fast_stop_decel_krpmps",      1000},
    {"fast_stop_decel_current",     -50},
    {"speed_ref_lpf",               650},
    {"f_speed_loop_kp",             300},
    {"f_speed_loop_ki",             59},
    {"s_speed_loop_kp",             150},
    {"s_speed_loop_ki",             29},
    {"speed_loop_anti_windup_enable", 0},
    {"speed_loop_anti_windup_coef", 1},

    // Throttle params
    {"ppm_period_tolerance",        1000},
    {"ppm_lost_time_ms",            300},
    {"hyst_ppm",                    100},
    {"throttle_type",               0},       // 0 = Normal
    {"ppm_curve_type",              1},       // 1 = Linear Speed
    {"normal_pwm_start",            10500},   // 1050us in 0.1us units
    {"normal_pwm_end",              19500},   // 1950us
    {"normal_pwm_curve_comp_coef",  3000},

    // Advance params
    {"noload_detect_enable",        1},
    {"load_dectect_speed_rpm",      750},
    {"load_detect_current",         80},
    {"field_weakening_enable",      1},
    {"field_weakening_max_current", -200},
    {"acc_comp_enable",             1},
    {"acc_comp_coef",               30},
    {"rs_temp_comp_enable",         1},
    {"rs_temp_comp_coef",           100},
    {"motor_sound_enable",          1},
    {"pndef_motor_sound_volume",    3},

    // Observer params
    {"observer_type",               0},
    {"observer_coef",               20},
    {"observer_filter_freq",        1000},

    // Protect params
    {"stall_enable",                1},
    {"stall_idle_speed_current",    400},
    {"stall_full_speed_current",    2200},
    {"stall_protected_duration",    1000},
    {"stall_count",                 2},
    {"stall_recover_enable",        0},
    {"overcurrent_count",           2},
    {"high_temp_protect_enable",    1},
    {"mos_high_temp_limit_1",       110},
    {"mos_high_temp_limit_2",       140},
    {"cap_high_temp_limit_1",       95},
    {"cap_high_temp_limit_2",       105},
    {"mcu_high_temp_limit_1",       100},
    {"mcu_high_temp_limit_2",       125},
    {"low_voltage_protect_enable",  0},
    {"low_protect_voltage_limit_1", 55},
    {"low_protect_voltage_limit_2", 35},
    {"low_protect_voltage_ratio",   40},
    {"ibus_max_current",            2000},
    {"ibus_limit_duration",         1000},
    {"max_power_limit",             25000},
    {"power_limit",                 1},

    // Bidirectional throttle params
    {"positive_ppm_start",          15000},
    {"positive_ppm_end",            20000},
    {"negative_ppm_start",          10000},
    {"negative_ppm_end",            15000},
    {"postive_pwm_curve_comp_coef", 1905},
    {"negative_pwm_curve_comp_coef", 1905},

    // Position mode params
    {"position_mode_enable",        0},
    {"position_mode_enable_work_type", 0},
    {"position_mode_speed_loop_kp", 680},
    {"position_mode_speed_loop_ki", 1500},
    {"position_mode_speed_loop_kd", 480},
    {"position_loop_pid_limit_1",   300},
    {"position_loop_pid_limit_2",   1},
    {"position_loop_kp",            30},
    {"position_loop_ki",            10},
    {"position_loop_kd",            30},
    {"position_mode_current_loop_coef", 20},
    {"position_mode_max_current",   200},
    {"position_mode_acc_krpms",     50},
    {"position_mode_speed_pll_freq", 1200},
};

static const uint16_t PRODUCTION_CONFIG_COUNT = sizeof(PRODUCTION_CONFIG) / sizeof(PRODUCTION_CONFIG[0]);

#endif // POWERPACK_PARAM_TABLE_H_
