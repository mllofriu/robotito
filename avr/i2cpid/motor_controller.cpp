
#include "motor_controller.hpp"

// The max value held by a int16_t
#define MAX_INT16 32767
// Try to keep this a power of two
// so the compiler can optimize division 
// by shifting
#define SCALE_FACTOR 128

#include <avr/io.h>

MotorController::MotorController(
    float kp, float ki, float max_e_ki,
    float control_period_s) :
        // Multiply the constants by the scale factor
        // so as to use some bits of the ints 
        // for fractional values
        // Normalize by period to keep the constants
        // independent of the control frequency
        m_kp(kp * SCALE_FACTOR / control_period_s),
        m_ki(ki * SCALE_FACTOR / control_period_s), 
        m_max_e_ki(max_e_ki * SCALE_FACTOR),
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

inline int16_t add_saturate(
    volatile int16_t& a, int16_t b, int16_t max = MAX_INT16,
    int16_t min = -MAX_INT16)
{
    if (b > 0){
        if (max - b > a)
            a += b;
        else
            a = max;
    } else {
        if (min - b < a)
            a += b;
        else
            a = min;
    }
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
    // Compute the expected amount of tics per control period
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
    int16_t s = m_accum_p_err / SCALE_FACTOR;
    add_saturate(s, m_accum_i_err / SCALE_FACTOR);
    return s;
}

void
MotorController::new_control_cycle() {
    // Reset pe to target * kp as the error is now target
    m_accum_p_err = m_target_p;
    // Add target * ki to the accumulated error * ki = ie
    // Watch for max_e_ki
    add_saturate(m_accum_i_err, m_target_i, m_max_e_ki, -m_max_e_ki);
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