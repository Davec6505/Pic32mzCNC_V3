; ========================================
; Test 03: 20-Segment Circle Path
; ========================================
; Purpose: Validate Phase 3 junction velocity optimization
; Expected: Continuous motion through all 20 corners (no stops)
; Measurement: Time circle completion with stopwatch
;
; Phase 3 Target Performance:
;   - 18° junctions: 961 mm/min (96% of programmed speed)
;   - Circle time: ~2.2 seconds (27% faster than Phase 2)
;   - Oscilloscope: Frequency never drops to zero at junctions
; ========================================

G21 ; millimeters
G90 ; absolute mode
G17 ; XY plane
G94 ; units per minute feed rate
M3 S1000 ; spindle on
; Safety: Raise to 5mm
G0 Z5   
; Move to start position (10mm, 0mm)
G0 X50 Y50 ; Lower to working height
G0 Z0 ; 20-segment circle approximation (radius = 10mm, center at origin)
; Feedrate: 1000 mm/min for junction testing
G1 X9.511 Y3.090 F2000
G1 X8.090 Y5.878
G1 X5.878 Y8.090
G1 X3.090 Y9.511
G1 X0.000 Y10.000
G1 X-3.090 Y9.511
G1 X-5.878 Y8.090
G1 X-8.090 Y5.878
G1 X-9.511 Y3.090
G1 X-10.000 Y0.000
G1 X-9.511 Y-3.090
G1 X-8.090 Y-5.878
G1 X-5.878 Y-8.090
G1 X-3.090 Y-9.511
G1 X0.000 Y-10.000
G1 X3.090 Y-9.511
G1 X5.878 Y-8.090
G1 X8.090 Y-5.878
G1 X9.511 Y-3.090
G1 X10.000 Y0.000
; Retract to safety height
G0 Z5
; Return to origin
G0 X0 Y0
M5 ; spindle off

; ========================================
; Validation Checklist:
; ========================================
; [ ] Start timer when circle starts
; [ ] Stop timer when circle completes
; [ ] Target: ~2.2 seconds (Phase 3)
; [ ] Compare: ~3.0 seconds (Phase 2 - stops at corners)
; [ ] Oscilloscope: Verify frequency never zero
; [ ] Position: Returns to (10.000, 0.000, 5.000)
; ========================================
