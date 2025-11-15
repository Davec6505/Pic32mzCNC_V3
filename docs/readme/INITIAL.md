documentation set with diagrams and initialization code.

CNC Motion Control Architecture
This README provides an overview of the CNC motion control system architecture using TMR2 and OCx modules with Bresenham interpolation.

Features
Uses TMR2 as a free-running timer for a shared time base at 100khz.

Multiple Output Compare (OCx) modules generate step pulses for CNC axes.

Bresenham interpolation coordinates multi-axis motion.

Dominant axis drives timing and scheduling of subordinate axes.

Modular design with clear separation of planning, stepping, and pulse scheduling.

Components
Component

Description

TMR2

Free-running 32-bit timer with 10 μ s tick

OCx (OC1–OC4)

Output Compare modules for X, Y, Z, A axes

OCx ISR

Interrupt Service Routine for pulse generation

Bresenham Logic

Interpolates subordinate axis steps

Segment Planner

Preloads motion segments and acceleration data

Operation
TMR2 runs continuously without interrupts.

Each OCx module uses TMR2 as its time base and triggers interrupts on compare match.

The dominant axis (with the highest step count) drives the segment clock via its OCx ISR.

Bresenham algorithm updates error terms and schedules subordinate axis steps.

Pulse widths and timings are managed by updating OCx compare registers.

Segment execution completes when all steps are issued.

Best Practices
Use a 32-bit timer (TMR2:TMR3) to avoid overflow issues.

Schedule compare values ahead of the current timer to prevent spurious interrupts.

Clear OCx interrupt flags promptly in each ISR.

Use modular structs to track segment and axis states.

Example ISR Skeleton
void __ISR(_OC1_VECTOR, IPL5SOFT) OC1Handler(void) {
    IFSxCLR = _IFSx_OC1IF_MASK; // Clear interrupt flag
    uint32_t now = TMR2;

    bresenham_step(&segment, axes, now);

    OC1R = now + segment.x_interval;
    OC1RS = OC1R + pulse_width;
}
For more detailed documentation, diagrams, or initialization code, please refer to the project wiki or contact the maintainer.

Apply revision
Reject
