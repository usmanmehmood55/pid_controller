# PID Controller

This repository contains an implementation of a PID (Proportional-Integral-Differential)
controller in C.

A PID controller is a closed loop control system that is widely used in engineering
applications for controlling continuous-time dynamic systems. The PID controller
calculates an error value as the difference between a measured process variable and a
desired set-point, then applies a control action based on proportional, integral, and
derivative terms (sometimes denoted as P, I, and D respectively) which are designed to
adjust the output in order to minimize the error over time.

This code demonstrates a simple PID controller that can be used to control a system that
is modeled by a mathematical function.

## Setup

- Clone this repository.
- Initialize the submodule by running `git submodule init` followed by `git submodule
  update`. This will download the ring_buffer submodule from GitHub.
- Compile the program by running `make` in the terminal. This will generate an 
  executable file called `pid_controller.exe`.
- All the PID related code in in [`main.c`](main.c)


## Implementation
The PID controller is implemented as a struct named `pid_controller_t`. This struct
contains the PID weights (proportional, integral, and differential response weight
constants), the constant data sampling rate, and a ring buffer for holding previous
errors. 
```c
typedef struct
{
    const double kP;           // proportional response weight constant
    const double kI;           // integral response weight constant
    const double kD;           // differential response weight constant
    const double time;         // constant data sampling rate in milliseconds
    ring_buffer  error_buffer; // buffer for holding previous errors
} pid_controller_t;
```

The following functions are used for PID computation:

- `proportional`: calculates the proportional response
- `integral`: calculates the integral response
- `differential`: calculates the differential response
- `pid_compute`: calculates the PID response based on those responses

In the function named `function`, a mathematical function is used to simulate a system. 
The goal is to achieve a certain output value, and the PID controller is used to adjust
the input value to the system to reach that goal.

```
┌───────┐                     ┌──────────────────┐                  ┌────────┐
│       │                     │                  │                  │        │
│ input ├────────►(+)────────►│     function     ├────────o────────►│ output │
│       │          ▲          │                  │        │         │        │
└───────┘          │          └──────────────────┘        │         └────────┘
                   │                                      │
                   │                                      │
                   │   ┌────────────────┐    ┌───────┐    │
                   │   │                │    │       │    │
                   └───┤ PID controller │◄───┤ error │◄───┘
                       │                │    │       │
                       └────────────────┘    └───────┘
```

The PID weights and other parameters can be adjusted to control the behavior of the system.