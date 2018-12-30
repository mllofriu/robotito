
#include "motor_controller.hpp"

MotorController::MotorController(
    int16_t kp, int16_t ki, int16_t max_e_ki,
    int16_t control_period_s) :
        m_kp(kp), m_ki(ki), m_max_e_ki(max_e_ki),
        m_control_period_s(control_period_s)
{
    set_target(0);
}

void 
MotorController::set_target(int16_t tics_per_sec)
{
    int16_t tics_per_period = 
        tics_per_sec * m_control_period_s;
    m_target = tics_per_period;
    // TODO: overflow?
    m_target_p = m_target * m_kp;
    m_target_i = m_target * m_ki;
}

int16_t
MotorController::get_control_signal() {
    // Use the last 8 bits for pwm - last bit is used for sign
    // TODO: overflow?
    return m_accum_p_err / 128 + m_accum_i_err / 128;
}

void
MotorController::new_control_cycle() {
    // Reset pe to target * kp as the error is now target
    m_accum_p_err = m_target_p;
    // Add target * ki to the accumulated error * ki = ie
    // Watch for max_e_ki
    if (m_target_i > 0) {
        if (m_accum_i_err < m_max_e_ki - m_target_i)
            m_accum_i_err += m_target_i;
        else 
            m_accum_i_err = m_max_e_ki;
    } else if (m_target_i < 0) {
        if (m_accum_i_err > -m_max_e_ki - m_target_i)
            m_accum_i_err += m_target_i;
        else 
            m_accum_i_err = -m_max_e_ki;
    } 
    // Reset the current count of tics
    m_current = 0;
}

void 
MotorController::encoder_update(bool a, bool b){
    // http://www.me.unm.edu/~starr/teaching/me470/quad.pdf

    // Other unrelated pins might be triggering the interrupt
    // only perform computations if one of the pins is involved
    if (a == m_quad_a && b == m_quad_b) 
        return;

    // b_old ^ a_new == 1 iff dir is pos
    bool dir = m_quad_a ^ b;

    // update the accumulated error
    // TODO: find better way of doing this
    if ((m_current < m_target && dir) || 
            (m_current > m_target && !dir)){
        m_accum_p_err -= m_kp;
        if (m_accum_i_err > -m_max_e_ki + m_ki)
            m_accum_i_err -= m_ki;
        else
            m_accum_i_err = -m_max_e_ki;
    } else if ((m_current >= m_target && dir) || 
            (m_current <= m_target && !dir)){
        m_accum_p_err += m_kp;
        if (m_accum_i_err < m_max_e_ki - m_ki)
            m_accum_i_err += m_ki;
        else
            m_accum_i_err = m_max_e_ki;
    }

    // update the current count
    if (dir) {
        m_current++;
    } else {
        m_current--;
    }

    // save new state as old
    m_quad_a = a;
    m_quad_b = b;
}

int16_t 
MotorController::get_curr_vel(){
    return m_current;
}

int16_t 
MotorController::get_accum_p_err(){
    return m_accum_p_err;
}

int16_t 
MotorController::get_accum_i_err(){
    return m_accum_p_err;
}