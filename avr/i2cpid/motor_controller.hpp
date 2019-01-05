#pragma once

#include <stdint.h>

///@brief A PI motor controller that uses feedback from
///       quadrature endoders, using solely integer math.
///       The constants, error and target are stored in
///       two-bytes ints. The control terms (kp*e) and 
///       (ki * error_sum) are updated on each tick of the
///       encoder.
class MotorController {
public:
    ///@brief The constructor
    ///@param kp The proportionality constant. It effectively multiplies
    ///       the error in tics per second to get a pwm signal [-255, 255]
    ///@param ki The integral constant. It effectively multiplies the
    ///       integral of the error to get a pwm signal [-255, 255].
    ///@param max_e_ki The maximum allowd value of (ki * sum_err). 
    ///       Expressed in the space of the control signal (0,255].
    ///       This prevents the controller from lagging behind due to too much 
    ///       accumulated error.
    ///@param control_period_s the control period in seconds. This is used
    ///       to normalize the constants and make them indep. of the control
    ///       frequency.                        
    MotorController(
        float kp, float ki, float max_e_ki,
        float control_period_s);

    ///@brief Set the target expressed in tics of the encoder per second.
    ///@param tics_per_sec The expected amount of tics per second. The sign
    ///       denotes direction.
    void set_target(int16_t tics_per_sec);

    ///@brief Get the control signal, i.e. kp * err + ki * sum_err
    ///@note  Returns a value between -255 and 255
    int16_t get_control_signal();

    ///@brief Reset the state of the control cycle. This should be called
    ///       once per control cycle, after getting the control signal.
    void new_control_cycle();

    ///@brief This function updates the counters associated with the encoders.
    ///@note This is usually called from within a pin-change interrupt routine.
    void encoder_update(bool a, bool b);

    /// Getters
    int16_t get_curr_vel();
    int16_t get_accum_p_err();
    int16_t get_accum_i_err();

private:
    /// The target tic counts for a control period 
    int16_t m_target{0};
    /// The tic counts for the current control period
    volatile int16_t m_current{0};
    /// The control period in seconds
    float m_control_period_s{0};
    /// The proportional constant of the PI controller
    int16_t m_kp{0};
    /// The integral constant of the PI controller
    int16_t m_ki{0};
    /// Cached value of the target times the prop constant
    int16_t m_target_p{0};
    /// Cached value of the target times the integral constant
    int16_t m_target_i{0};
    /// Maximum allowed e * ki
    int16_t m_max_e_ki{0};
    /// The accumulated error times the prop constant
    int16_t m_pe{0};
    /// The accumulated error times the integral constant
    int16_t m_ie{0};
    /// The state of the encoders
    bool m_quad_a, m_quad_b;
    /// The accumulated error times the prop constant
    volatile int16_t m_accum_p_err{0};
    /// The accumulated error times the integral constant
    volatile int16_t m_accum_i_err{0};
};