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

#include <stdio.h>
#include <math.h>
#include "ring_buffer/ring_buffer.h"

#define UNUSED (void)

/**
 * @brief Contains the PID weights, and other data needed for
 * PID computation.
 */
typedef struct
{
    const double kP;           // proportional response weight constant
    const double kI;           // integral response weight constant
    const double kD;           // differential response weight constant
    const double time;         // constant data sampling rate in milliseconds
    ring_buffer  error_buffer;  // buffer for holding previous errors
} pid_controller_t;


/**
 * @brief Calculates the proportional response
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double proportional response
 */
static double proportional(double error, pid_controller_t * p_pid)
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
static double integral(double error, pid_controller_t * p_pid)
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
static double differential(double error, pid_controller_t * p_pid)
{
    double previous_error = 0.0;
    uint16_t prev_error_index = 
        (p_pid->error_buffer.head == 0U) ? (p_pid->error_buffer.size - 1U) : (p_pid->error_buffer.head- 1U);

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
static double pid_compute(double error, pid_controller_t * p_pid)
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

/**
 * @brief This can be the function that represents a mathematical model of
 * the system or an estimation of it. A transfer function. 
 * 
 * @param input transfer function's input
 * 
 * @return double transfer function output
 */
static double function(double input)
{
    return (2.0 * input) + 1.0;
}

int main(void)
{
    int iterations   = 0;
    double input     = 0.0;
    double goal      = 5.0;
    double output    = 0.0;
    double error     = 0.0;
    double pid_value = 0.0;
    
    pid_controller_t pid =
    {
        .kP   = 0.6,
        .kI   = 0.1,
        .kD   = 0.1,
        .time = 100,
    };

    // initialize the error buffer
    (void)ring_buffer_create(&pid.error_buffer, 30U);

    printf("\r%lf\n", output);

    // while convergence is not reached and iterations are within limit
    while ((fabs(goal - output) > 0.0001) && (iterations < 1000))
    {
        // get the output by giving input to the transfer function
        output = function(input);

        // calculate the error based on output and goal
        error = goal - output;

        // calculate the PID value based on the error
        pid_value = pid_compute(error, &pid);

        // add the PID value to the next input
        input += pid_value;
        
        printf("\r%lf\n", output);

        iterations++;
    }

    printf("\rConverged in %d iterations\n", iterations);

    // release the memory used by error buffer
    (void)ring_buffer_destroy(&pid.error_buffer);

    return 0;
}