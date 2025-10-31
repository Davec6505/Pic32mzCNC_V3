# Project Introduction
Pic32mzCNC_V3 is a modular CNC motion control system designed for high-performance, multi-axis stepper motor control using Microchip PIC32MZ microcontrollers. It features precise pulse scheduling, Bresenham interpolation, and a flexible architecture suitable for custom CNC machines and automation projects.

## Hardware & Toolchain Requirements
- **Microcontroller:** Microchip PIC32MZ (e.g., PIC32MZ2048EFH100)
- **Development Tools:** MPLAB X IDE, XC32 Compiler
- **Supported Platforms:** Custom CNC hardware with stepper drivers
- **Other:** Standard build environment with `make` utility

## Quick Start / Build Instructions
1. Clone the repository:
  ```sh
  git clone https://github.com/Davec6505/Pic32mzCNC_V3.git
  ```
2. Build the project:
  ```sh
  make
  ```
3. Flash the firmware to your PIC32MZ device:
  ```
  MikroE bootloader.
  ```
4. Clean build artifacts:
  ```sh
  make clean
  ```

Refer to the documentation and source code for further details on configuration and usage.

# Pic32mzCNC_V3
CNC Motion Control Architecture

This README provides an overview of the CNC motion control system architecture using TMR2 and OCx modules with Bresenham interpolation.

FEATURES:
Uses TMR2 as a free-running 32bit timer @10us/tick no ISR.
Multiple Output Compare (OCx) using TMR2 as the shared base timer for all modules to generate step pulses for CNC axes.
OCx set up for use in continous pulse mode, sliding scale [OCxR:OCxRS] for pulse generation.
Bresenham interpolation coordinates multi-axis motion, will run from a state machine not ISR driven, OCx ISR modules will be used for counting and kept as short as possible.
Dominant axis drives timing and scheduling of subordinate axes, dominant axis is set by changing [OCxR:OCxRS] pair by placing a values that are ahead of TMR2's value, subordinate axis will use the same method, but value will only be set ahead of TMR during Bresenham error check otherwise left at current vaalue.



Modular design with clear separation of planning, stepping, and pulse scheduling.

COMPONENTS:                              
---------------------------------------------------------------------------------------------
|________________Component_____________|__________________Description_______________________|
|                   TMR2               |    Free-running 16-bit timer with 10 μ s tick      |
|               OCx (OC1–OC4)          |    Output Compare modules for X, Y, Z, A axes      |
|                   OCx ISR            |    Interrupt Service Routine for pulse generation  |
!               Bresenham Logic        |    Interpolates subordinate axis steps             |
|               Segment Planner        |    Preloads motion segments and acceleration data  |
---------------------------------------------------------------------------------------------

OPERATION:
TMR2 runs continuously without interrupts.
Each OCx module uses TMR2 as its time base and triggers interrupts on compare match.
The dominant axis (with the highest step count) drives the segment clock via its OCx ISR.
Bresenham algorithm updates error terms and schedules subordinate axis steps.
Pulse widths and timings are managed by updating OCx compare registers.
Segment execution completes when all steps are issued.

BEST PRACTICES:
Use a 32-bit timer (TMR2:TMR3) to avoid overflow issues.
Choose one of the methods below for flow control.
******************************************************************************************
?=Stop Start TMR2 at will but ensure OCxR & OCxRS are at 0 and force TMR to start from 1.
?=Stop Start OCx modules.
******************************************************************************************
Schedule compare values ahead of the current timer to prevent spurious interrupts.
Clear OCx interrupt flags promptly in each ISR, keep ISR's short.
Use modular structs to track segment and axis states.

Example ISR Skeleton
void __ISR(_OC1_VECTOR, IPL5SOFT) OC1Handler(void) {
    IFSxCLR = _IFSx_OC1IF_MASK; // Clear interrupt flag

    bresenham_step(&segment, axes, now);
  
  Possibly do this in Bresenhams calculation for better control.
  *  uint32_t now = TMR2;
  *  OC1R = now + segment.x_interval;
  *  OC1RS = OC1R + pulse_width;
}