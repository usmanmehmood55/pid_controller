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


### Pre Requisites

The required software to use this library are
- [Git](https://git-scm.com/downloads)
- [Make](https://gnuwin32.sourceforge.net/packages/make.htm)


### Downloading and Compilation
- Clone this repository using
  ```bash
  git clone https://github.com/usmanmehmood55/pid_controller.git
  ```
- Initialize the ring_buffer submodule by running
  ```bash
  git submodule init
  git submodule update
  ```
- Compile the program by running `make` in the terminal. This will generate an
  executable file called `pid_controller.exe`.
  ```bash
  make
  ```


### Execution
- Run the executable using
  ```bash
  ./pid_controller.exe
  ```

### Updating
- Before updating be sure to save any changes that were made to the PID code as they
  might get overwritten.
- To get the latest code from this repository, simply `pull` from Git. 
  ```bash
  git pull
  ```


## Implementation
All the PID related code in in [`main.c`](main.c). The PID controller is implemented
as a struct named `pid_controller_t`. This struct contains the PID weights 
(proportional, integral, and differential response weight constants), the constant
data sampling rate, a ring buffer for holding previous errors, a function for giving
input to the PID controller, and a function to act as the transfer function of the
system.
```c
typedef struct
{
    const double  kP;               // proportional response weight constant
    const double  kI;               // integral response weight constant
    const double  kD;               // differential response weight constant
    const double  time;             // constant data sampling rate in milliseconds
    const double  goal;             // goal / set-point for PID controller
    ring_buffer_t * p_error_buffer; // buffer for holding previous errors

    double (*get_input)(double previous_input); // input function's pointer
    double (*transfer_function)(double input);  // transfer function's pointer

} pid_controller_t;
```

The following functions are used for PID computation:

- `proportional`: calculates the proportional response
- `integral`: calculates the integral response
- `differential`: calculates the differential response
- `pid_compute`: calculates the PID response based on those responses

In the function named `transfer_function`, a mathematical function is used to simulate 
a system. The goal is to achieve a certain output value, and the PID controller is used
to adjust the input value to the system to reach that goal.

```
+-------+                 +----------------+      +-------------------+              +--------+
|       |                 |                |      |                   |              |        |
| input |------>(+)------>| PID controller |----->| transfer_function |------o------>| output |
|       |        ^        |                |      |                   |      |       |        |
+-------+        |        +----------------+      +-------------------+      |       +--------+
                 |                                                           |
                 |                                                           |
                 |                         +-------+                         |
                 |                         |       |                         |
                 +-------------------------| error |<------------------------+
                                           |       |
                                           +-------+
```

The PID weights and other parameters can be adjusted to control the behavior of the system.