#pragma once

#include <stdint.h>

class MotorController {
public:
    MotorController(
        int16_t kp, int16_t ki, int16_t max_e_ki,
        int16_t control_period_s);

    void set_target(int16_t tics_per_sec);

    int16_t get_control_signal();

    void new_control_cycle();

    void encoder_update(bool a, bool b);

    int16_t get_curr_vel();
    int16_t get_accum_p_err();
    int16_t get_accum_i_err();

private:
    // The target tic counts for a control period 
    int16_t m_target{0};
    // The tic counts for the current control period
    int16_t m_current{0};
    // The control period in seconds
    int16_t m_control_period_s{0};
    // The proportional constant of the PI controller
    int16_t m_kp{0};
    // The integral constant of the PI controller
    int16_t m_ki{0};
    // Cached value of the target times the prop constant
    int16_t m_target_p{0};
    // Cached value of the target times the integral constant
    int16_t m_target_i{0};
    // Maximum allowed e * ki
    int16_t m_max_e_ki{0};
    // The accumulated error times the prop constant
    int16_t m_pe{0};
    // The accumulated error times the integral constant
    int16_t m_ie{0};
    // The state of the encoders
    bool m_quad_a, m_quad_b;
    // The accumulated error times the prop constant
    int16_t m_accum_p_err{0};
    // The accumulated error times the integral constant
    int16_t m_accum_i_err{0};
};