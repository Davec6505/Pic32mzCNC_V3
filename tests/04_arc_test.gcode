; ========================================
; Test 04: Arc Interpolation (G2/G3)
; ========================================
; Purpose: Validate arc-to-segment conversion algorithm
; Expected: Arcs converted to linear segments automatically
; Measurement: Compare segment count vs. linear approximation
;
; Arc Algorithm (GRBL-style):
;   - Default $12 = 0.002mm arc tolerance
;   - Segments = floor(0.5 * angle * radius / sqrt(tolerance * (2*radius - tolerance)))
;   - Example: 90° arc @ 10mm radius ≈ 28 segments
; ========================================

G21 ; millimeters
G90 ; absolute mode
G17 ; XY plane
G94 ; units per minute feed rate
M3 S1000 ; spindle on

; Safety: Raise to 5mm
G0 Z5 F1500

; Move to start position (0mm, 0mm)
G0 X0 Y0

; Lower to working height
G0 Z0

; ========================================
; Test 1: Quarter Circle CW (G2)
; ========================================
; Start: (0, 0)
; End: (10, 0)
; Center: (5, 0) via I5 J0
; Expected: 90° arc, CW, radius 5mm, ~18 segments
G2 X10 Y0 I5 J0 F1000

; Pause at corner
G4 P0.5

; ========================================
; Test 2: Quarter Circle CCW (G3)
; ========================================
; Start: (10, 0)
; End: (10, 10)
; Center: (10, 5) via I0 J5
; Expected: 90° arc, CCW, radius 5mm, ~18 segments
G3 X10 Y10 I0 J5 F1000

; Pause at corner
G4 P0.5

; ========================================
; Test 3: Semicircle CW (G2)
; ========================================
; Start: (10, 10)
; End: (0, 10)
; Center: (5, 10) via I-5 J0
; Expected: 180° arc, CW, radius 5mm, ~35 segments
G2 X0 Y10 I-5 J0 F1000

; Pause at corner
G4 P0.5

; ========================================
; Test 4: Full Circle CW (G2)
; ========================================
; Start: (0, 10)
; End: (0, 10) - same as start
; Center: (0, 0) via I0 J-10
; Expected: 360° arc, CW, radius 10mm, ~70 segments
; NOTE: Full circles need special handling (not yet implemented)
; This test will fail with "error: G2/G3 requires I,J,K or R"
; G2 X0 Y10 I0 J-10 F1000

; ========================================
; Test 5: Large Radius Arc
; ========================================
; Start: (0, 10)
; End: (20, 10)
; Center: (10, 10) via I10 J0
; Expected: 180° arc, CW, radius 10mm, ~70 segments
G2 X20 Y10 I10 J0 F1000

; Retract to safety height
G0 Z5

; Return to origin
G0 X0 Y0

M5 ; spindle off

; ========================================
; Validation Checklist:
; ========================================
; [ ] Test 1: Machine moves from (0,0) to (10,0) in smooth arc
; [ ] Test 2: Machine moves from (10,0) to (10,10) in smooth arc
; [ ] Test 3: Machine moves from (10,10) to (0,10) in semicircle
; [ ] Test 4: EXPECTED TO FAIL (full circle not implemented)
; [ ] Test 5: Machine moves from (0,10) to (20,10) in large arc
; [ ] Position: Returns to (0.000, 0.000, 5.000)
; [ ] Serial Output: Check for segment count messages
; [ ] Oscilloscope: Verify smooth velocity profile
; ========================================
