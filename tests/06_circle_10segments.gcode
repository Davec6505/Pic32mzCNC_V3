; Test 6: Circle with 10 segments (Half of original)
; Purpose: Test if smaller number of moves works better than 20
; Expected: Should trace half circle (180 degrees)
; Path: 20mm diameter circle, 10 segments instead of 20

G21         ; Metric mode (mm)
G90         ; Absolute positioning
G17         ; XY plane
G94         ; Units per minute feedrate mode
M3 S1000    ; Spindle on CW at 1000 RPM

G0 Z5       ; Lift Z to safe height
G0 X10 Y0   ; Move to start position (right side of circle)
G0 Z0       ; Lower Z to work height

; Half circle (10 segments, 18° each, counterclockwise from right)
G1 X9.511 Y3.090 F1000   ; 18°
G1 X8.090 Y5.878         ; 36°
G1 X5.878 Y8.090         ; 54°
G1 X3.090 Y9.511         ; 72°
G1 X0.000 Y10.000        ; 90° (top of circle)
G1 X-3.090 Y9.511        ; 108°
G1 X-5.878 Y8.090        ; 126°
G1 X-8.090 Y5.878        ; 144°
G1 X-9.511 Y3.090        ; 162°
G1 X-10.000 Y0.000       ; 180° (left side of circle)

; Return to start
G0 Z5       ; Lift Z
G0 X0 Y0    ; Return to origin
M5          ; Spindle off

; VERIFICATION:
; [ ] Should execute 10 moves smoothly
; [ ] Final position before return: (-10.000, 0.000, 5.000)
; [ ] Path should be smooth arc, not diagonal line
; [ ] Compare with 20-segment version to isolate buffer size issue
