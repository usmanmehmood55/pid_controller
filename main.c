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
#include "pid_control/pid_control.h"

static double get_input(double input)
{
    return input;
}

static double transfer_function(double input)
{
    return (2.0 * input) + 1.0;
}

int main(void)
{
    pid_controller_t pid =
    {
        .kP   = 0.5,
        .kI   = 0.4,
        .kD   = 0.3,
        .time = 100,
        .goal = 200.0,
        .get_input         = get_input,
        .transfer_function = transfer_function,
    };

    double output = pid_controller(pid);
    printf("\routput: %lf\n", output);

    return 0;
}