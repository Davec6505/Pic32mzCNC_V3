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
  
2. Build the project:
  ```sh
  make  / make all = clean and build
   
3. Flash the firmware to your PIC32MZ device:
  ```sh
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


CONSIDERATIONS:
## TMR2 is free running.
### Option 3: Use 32-bit TMR2:TMR3 Pair
1. 	Gives you a much larger range (up to 4.29 billion ticks).
2. 	Reduces the chance of overflow or wraparound issues.
3. 	Lets you schedule OCxR values far ahead without worrying about PR2.
4.  [OCxR:OCxRS] in absolute mode.


There are two alternative initial conditions to consider:  
1. Initialize TMRy = 0 and set OCxR ≥ 1.  
2. Initialize TMRy = PRy (PRy > 0) and set OCxR = 0. | Pulse will be delayed by the value in the PRy register, depending on setup |

## Special Cases for Dual Compare Mode (Single Output Pulse)

### 1. PRy ≥ OCxRS and OCxRS > OCxR
- **Setup:** OCxR = 0, TMRy = 0
- **Operation:**
  - No pulse on first timer cycle; OCx pin stays low.
  - After TMRy resets (period match), OCx pin goes high at OCxR match.
  - At next OCxRS match, OCx pin goes low and stays low.
  - OCxIF interrupt flag set on second compare.
- **Alternative Initializations:**
  - TMRy = 0, OCxR ≥ 1
  - TMRy = PRy (>0), OCxR = 0
- **Output:** Pulse is delayed by PRy value.

### 2. PRy ≥ OCxR and OCxR ≥ OCxRS
- **Setup:** OCxR ≥ 1, PRy ≥ 1
- **Operation:**
  - TMRy counts to OCxR, OCx pin goes high.
  - TMRy continues, resets at PRy (period match).
  - Counts to OCxRS, OCx pin goes low.
  - OCxIF interrupt flag set on second compare.
- **Output:** Standard pulse.

### 3. OCxRS > PRy and PRy ≥ OCxR
- **Setup:** None
- **Operation:**
  - Only rising edge generated at OCx pin.
  - OCxIF interrupt flag not set.
- **Output:** Pin transitions high (rising edge only).

### 4. OCxR > PRy
- **Setup:** None
- **Operation:**
  - Unsupported mode; timer resets before match.
- **Output:** Pin remains low.

**Legend:**
- OCxR: Compare Register
- OCxRS: Secondary Compare Register
- PRy: Timer Period Register
- TMRy: Timer Count

**Note:** TMRy is assumed to be initialized to 0x0000 in all cases.
