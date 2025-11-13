; Test 1: Simple 90° Junction
; Purpose: Verify continuous motion through right-angle corner
; Expected: Machine flows through corner at ~707mm/min (71% of 1000mm/min)
; Verification: Oscilloscope shows frequency slows but never drops to zero
; Duration: ~2 seconds

G21         ; Metric mode (mm)
G90         ; Absolute positioning
G17         ; XY plane
G92 X0 Y0 Z0 ; Zero all axes
G1 F1000    ; Set feedrate to 1000mm/min

; Move X-axis 10mm
G1 X10.000

; 90° junction - should NOT stop!
G1 Y10.000

; Return to origin
G1 X0.000
G1 Y0.000

; Test complete
M0 ; Program stop

; VERIFICATION CHECKLIST:
; [ ] Total time: ~2 seconds (was 2.5s if stopping at corners)
; [ ] Oscilloscope: X-axis step frequency slows at junction but never zero
; [ ] Junction speed: ~707mm/min (measure frequency: ~9.4kHz at junction)
; [ ] Final position: (0.000, 0.000) - machine returns to start
