; Test 2: Multiple 90° Corners (Rectangle)
; Purpose: Verify continuous motion through multiple junctions
; Expected: Flows through all 4 corners without stopping
; Verification: Total time ~4 seconds (vs 5+ seconds if stopping)
; Path: 20mm x 10mm rectangle

G21         ; Metric mode (mm)
G90         ; Absolute positioning
G17         ; XY plane
G92 X0 Y0 Z0 ; Zero all axes
G1 F500    ; Set feedrate to 1000mm/min

; Rectangle path (4 corners)
G1 X20.000 Y0.000   ; Move to corner 1
G1 X20.000 Y10.000  ; Corner 2 (90° junction)
G1 X0.000 Y10.000   ; Corner 3 (90° junction)
G1 X0.000 Y0.000    ; Corner 4 (90° junction) - back to start
; Rectangle path (4 corners)
G1 X20.000 Y0.000   ; Move to corner 1
G1 X20.000 Y10.000  ; Corner 2 (90° junction)
G1 X0.000 Y10.000   ; Corner 3 (90° junction)
G1 X0.000 Y0.000    ; Corner 4 (90° junction) - back to start

G1 X20.000 Y10.000 ; Move diagonal
G1 X0.000   ; Move X back to 0
G1 Y0.000   ; Move Y back to 0

; Test complete
M0 ; Program stop

; VERIFICATION CHECKLIST:
; [ ] Total time: ~4 seconds (20mm + 10mm + 20mm + 10mm = 60mm @ 1000mm/min)
; [ ] No stops at any of the 4 corners
; [ ] Oscilloscope: Frequency varies but never zero
; [ ] Final position: (0.000, 0.000) - returns to origin
; [ ] Path accuracy: Measure rectangle dimensions with calipers
;     - X dimension: 20.000mm ±0.1mm
;     - Y dimension: 10.000mm ±0.1mm
