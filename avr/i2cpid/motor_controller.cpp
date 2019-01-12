
#include "motor_controller.hpp"

// Try to keep this a power of two
// so the compiler can optimize division 
// by shifting
#define SCALE_FACTOR 128
// The max value of the output signal (255) scaled 
// by the scale factor
// There is no point on accumulators going over this value
#define MAX_SIGNAL 32640

#define MAX_SBYTE 127
#define MAX_INT16 32767

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
    volatile int16_t& a, int16_t b, int16_t max = MAX_SIGNAL,
    int16_t min = -MAX_SIGNAL)
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

inline int8_t add_saturate(
    volatile int8_t& a, int8_t b, int8_t max = MAX_SBYTE,
    int16_t min = -MAX_SBYTE)
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

void 
MotorController::set_target(int16_t tics_per_period)
{
    m_target = tics_per_period;
    // TODO: overflow?
    m_target_p = max(-MAX_SIGNAL, min(m_target * m_kp, MAX_SIGNAL));
    m_target_i = max(-MAX_SIGNAL, min(m_target * m_ki, MAX_SIGNAL));
}

int16_t
MotorController::get_control_signal() {
    if (!m_enabled)
        m_last_control_signal = 0;
    else {
        // Use the last 8 bits for pwm - last bit is used for sign
        int16_t s = m_accum_p_err;
        add_saturate(s, m_accum_i_err);
        m_last_control_signal = s / SCALE_FACTOR;
    }
    return m_last_control_signal;
}

int16_t
MotorController::get_last_control_signal(){
    return m_last_control_signal;
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
    // compute the update to p and i accum k*err
    int16_t p_upd, i_upd;
    int8_t tick_upd;
    if (dir) {
        p_upd = -m_kp;
        i_upd = -m_ki;
        tick_upd = 1;
    } else {
        p_upd = m_kp;
        i_upd = m_ki;
        tick_upd = -1;
    }
    // Update the accum k*err
    add_saturate(m_accum_p_err, p_upd);
    add_saturate(m_accum_i_err, i_upd,  m_max_e_ki, -m_max_e_ki);
    add_saturate(m_current, tick_upd, MAX_SBYTE, -MAX_SBYTE);
    add_saturate(m_accum_ticks, tick_upd, MAX_INT16, -MAX_INT16);
    
    // save new state
    m_quad_a = a;
    m_quad_b = b;
}

int16_t 
MotorController::get_accum_ticks(){
    int16_t val = m_accum_ticks;
    m_accum_ticks = 0;
    return val;
}

int16_t 
MotorController::get_accum_p_err(){
    return m_accum_p_err;
}

int16_t 
MotorController::get_accum_i_err(){
    return m_accum_p_err;
}

void 
MotorController::enable(){
    m_enabled = true;
}

void 
MotorController::disable(){
    m_enabled = false;
}