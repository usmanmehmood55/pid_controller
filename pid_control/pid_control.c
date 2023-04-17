/**
 * @file    main.c
 * @author  Usman Mehmood (usmanmehmood55@gmail.com)
 * @brief   This file implements a PID controller
 * 
 * @version 0.1
 * @date    15-04-2023
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "pid_control.h"

#define UNUSED (void)

/**
 * @brief Calculates the proportional response
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double proportional response
 */
double proportional(double error, pid_controller_t * p_pid)
{
    UNUSED(p_pid);

    // this can be a negative feedback by making it return -error
    return error;
}

/**
 * @brief Calculates the integral response
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double integral response
 */
double integral(double error, pid_controller_t * p_pid)
{
    (void)ring_buffer_add(&p_pid->error_buffer, error);
    double sum = 0.0;
    (void)ring_buffer_sum(&p_pid->error_buffer, &sum);
    return sum;
}

/**
 * @brief Calculates the differential response
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double differential response
 */
double differential(double error, pid_controller_t * p_pid)
{
    double previous_error = 0.0;
    uint16_t prev_error_index = 
        (p_pid->error_buffer.head == 0U) ? (p_pid->error_buffer.size - 1U) : (p_pid->error_buffer.head - 1U);

    (void)ring_buffer_get(&p_pid->error_buffer, prev_error_index, &previous_error);

    double delta = error - previous_error;
    double diff = delta / p_pid->time;

    return diff;
}

/**
 * @brief Calculates the PID response based on current error
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double PID response
 */
double pid_compute(double error, pid_controller_t * p_pid)
{
    double prop         = 0.0;
    double integ        = 0.0;
    double diff         = 0.0;
    double prop_factor  = 0.0;
    double integ_factor = 0.0;
    double diff_factor  = 0.0;
    double pid_value    = 0.0;

    prop  = proportional(error, p_pid);
    integ = integral(error, p_pid);
    diff  = differential(error, p_pid);

    prop_factor  = p_pid->kP * (prop);
    integ_factor = p_pid->kI * (integ);
    diff_factor  = p_pid->kD * (diff);

    pid_value = prop_factor + integ_factor + diff_factor;

    return pid_value;
}

double pid_controller(pid_controller_t pid)
{
    
    double input     = 0.0;
    double goal      = 5.0;
    double output    = 0.0;
    double error     = 0.0;
    double pid_value = 0.0;

    // initialize the error buffer
    (void)ring_buffer_create(&pid.error_buffer, 30U);

    printf("\r%lf\n", output);

    // while convergence is not reached and iterations are within limit
    int iterations = 0;
    while ((fabs(goal - output) > 0.000001) && (iterations < 1000))
    {
        // summing junction, error is added to input
        input += error;

        // error + input is given to PID control function
        pid_value = pid_compute(input, &pid);

        // PID value is given to transfer function to get output
        output = transfer_function(pid_value);

        // error is calculated from output
        error = goal - output;

        printf("\r%lf\n", output);

        iterations++;
    }

    printf("\rConverged in %d iterations\n", iterations);

    // release the memory used by error buffer
    // (void)ring_buffer_destroy(&pid.error_buffer);

    return output;
}