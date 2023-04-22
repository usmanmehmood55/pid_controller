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
#include <stdbool.h>
#include "../ring_buffer/ring_buffer.h"

/**
 * @brief Contains the PID weights, and other data needed for
 * PID computation.
 */
typedef struct
{
    const double  kP;               // proportional response weight constant
    const double  kI;               // integral response weight constant
    const double  kD;               // differential response weight constant
    const double  time;             // constant data sampling rate in milliseconds
    const double  goal;             // goal / set-point for PID controller
    ring_buffer_t * p_error_buffer; // buffer for holding previous errors

    double (*get_input)(double input);         // input function's pointer
    double (*transfer_function)(double input); // transfer function's pointer

} pid_controller_t;

/**
 * @brief Calculates the proportional response
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double proportional response
 */
double proportional(double error, pid_controller_t * p_pid);

/**
 * @brief Calculates the integral response
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double integral response
 */
double integral(double error, pid_controller_t * p_pid);

/**
 * @brief Calculates the differential response
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double differential response
 */
double differential(double error, pid_controller_t * p_pid);

/**
 * @brief Calculates the PID response based on current error
 * 
 * @param error current error
 * @param p_pid pointer to PID object
 * 
 * @return double PID response
 */
double pid_compute(double error, pid_controller_t * p_pid);

double pid_controller(pid_controller_t pid);