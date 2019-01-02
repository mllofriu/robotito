
#include "motor_controller.hpp"

#define MAX_INT16 32767

#include <avr/io.h>

MotorController::MotorController(
    int16_t kp, int16_t ki, int16_t max_e_ki,
    float control_period_s) :
        m_kp(kp), m_ki(ki), m_max_e_ki(max_e_ki),
        m_control_period_s(control_period_s)
{
    set_target(0);
}

int16_t min(int16_t a, int16_t b)
{
    if (a < b)
        return a;
    else
        return b;
}

int16_t max(int16_t a, int16_t b)
{
    if (a > b)
        return a;
    else
        return b;
}

inline int16_t add_saturate(volatile int16_t& a, int16_t b, int16_t max = MAX_INT16)
{
    if (max - b > a)
        a += b;
    else
        a = max;
}

inline int16_t sub_saturate(volatile int16_t& a, int16_t b, int16_t min = -MAX_INT16)
{
    if (min + b < a)
        a -= b;
    else
        a = min;
}

void 
MotorController::set_target(int16_t tics_per_sec)
{
    int16_t tics_per_period = 
        tics_per_sec * m_control_period_s;
    m_target = tics_per_period;
    // TODO: overflow?
    m_target_p = max(-MAX_INT16, min(m_target * m_kp, MAX_INT16));
    m_target_i = max(-MAX_INT16, min(m_target * m_ki, MAX_INT16));
}

int16_t
MotorController::get_control_signal() {
    // Use the last 8 bits for pwm - last bit is used for sign
    return m_accum_p_err / 128 + m_accum_i_err / 128;
    //return m_kp * (m_current - m_target) / 128;
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
    bool dir = m_quad_b ^ a;

    // update the current count
    if (dir) {
        sub_saturate(m_accum_p_err, m_kp);
        sub_saturate(m_accum_i_err, m_ki, -m_max_e_ki);
        m_current++;
        PORTB |= 1 << PB6;
    } else {
        add_saturate(m_accum_p_err, m_kp);
        add_saturate(m_accum_i_err, m_ki, m_max_e_ki);
        m_current--;
        PORTB &= ~(1 << PB6);
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