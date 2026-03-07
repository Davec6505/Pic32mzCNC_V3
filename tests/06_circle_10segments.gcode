; Test 6: Circle with 10 segments (Half of original)
; Purpose: Test if smaller number of moves works better than 20
; Expected: Should trace half circle (180 degrees)
; Path: 20mm diameter circle, 10 segments, centred at X20 Y10 to stay in positive work area

G21         ; Metric mode (mm)
G90         ; Absolute positioning
G17         ; XY plane
G94         ; Units per minute feedrate mode
M3 S1000    ; Spindle on CW at 1000 RPM

G0 Z5       ; Lift Z to safe height
G0 X30 Y10  ; Move to start position (right side of circle, centre at X20 Y10)
G0 Z10      ; Lower Z to work height

; Half circle (10 segments, 18° each, counterclockwise from right)
; Centre X20 Y10, radius 10mm — all points remain X>=10
G1 X29.511 Y13.090 F1000  ; 18°
G1 X28.090 Y15.878        ; 36°
G1 X25.878 Y18.090        ; 54°
G1 X23.090 Y19.511        ; 72°
G1 X20.000 Y20.000        ; 90° (top of circle)
G1 X16.910 Y19.511        ; 108°
G1 X14.122 Y18.090        ; 126°
G1 X11.910 Y15.878        ; 144°
G1 X10.489 Y13.090        ; 162°
G1 X10.000 Y10.000        ; 180° (left side of circle)

; Return to start
G0 Z5       ; Lift Z
G0 X0 Y0    ; Return to origin
M5          ; Spindle off

; VERIFICATION:
; [ ] Should execute 10 moves smoothly
; [ ] All X positions >= 10 — no limit trigger
; [ ] Final position before return: (10.000, 10.000, 5.000)
; [ ] Path should be smooth arc
