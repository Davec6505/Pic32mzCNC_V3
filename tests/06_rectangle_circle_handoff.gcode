; Rectangle-to-Circle Handoff Test
; Based on user's image showing rectangle with rounded corners connecting to circle
; Tests linear-to-arc transitions and arc-to-linear transitions
;
; Test Pattern:
; 1. Rectangle with rounded (filleted) corners
; 2. Transition to circular path
; 3. Return via circle back to rectangle
;
; Origin: Lower-left of rectangle
; Rectangle: 20mm × 30mm with 3mm radius corners
; Circle: 20mm diameter (10mm radius) centered at (30, 15)
;
; Created: October 26, 2025
; Status: Ready for testing after subordinate axis fix

; Initialize
G21           ; Millimeters
G90           ; Absolute positioning
G94           ; Units per minute feedrate mode
F200         ; Set feedrate to 300 mm/min

; Zero position
G92 X0 Y0 Z0

; Lift to safe height
G0 Z5

; Move to start position (bottom-left, 3mm from corner for fillet)
G0 X3 Y0

; Lower to work height
G0 Z0

; =====================================
; RECTANGLE WITH ROUNDED CORNERS (20mm × 30mm, R3 corners)
; =====================================

; Bottom edge (left fillet to right fillet)
G1 X17 Y0 F1000

; Bottom-right corner (90° arc, R3)
G2 X20 Y3 I0 J3

; Right edge (bottom to top)
G1 X20 Y27

; Top-right corner (90° arc, R3)
G2 X17 Y30 I-3 J0

; Top edge (right to left)
G1 X3 Y30

; Top-left corner (90° arc, R3)
G2 X0 Y27 I0 J-3

; Left edge (top to bottom)
G1 X0 Y3

; Bottom-left corner (90° arc, R3)
G2 X3 Y0 I3 J0

; =====================================
; TRANSITION: RECTANGLE TO CIRCLE
; =====================================

; Move from rectangle (3, 0) to circle start (40, 15)
; This is the transition line connecting rectangle to circle
G1 X40 Y15

; =====================================
; CIRCULAR PATH (20mm diameter, center at (30, 15))
; =====================================

; Full circle starting at (40, 15) - split into two 180° arcs
; Arc 1: Right semicircle (40,15) → (20,15) center (30,15)
G2 X20 Y15 I-10 J0

; Arc 2: Left semicircle (20,15) → (40,15) center (30,15)
G2 X40 Y15 I10 J0

; =====================================
; TRANSITION: CIRCLE BACK TO RECTANGLE
; =====================================

; Return to rectangle starting point
G1 X3 Y0

; Lift to safe height
F200
G0 Z5

; Move to origin
F1000
G0 X0 Y0

; Program end
M30

; =====================================
; EXPECTED RESULTS:
; =====================================
; - Rectangle with smooth rounded corners (no sharp 90° turns)
; - Clean transition from rectangle to circle
; - Full circular path
; - Return to starting position
;
; CHECKS:
; - Final position should be (0, 0, 5) - verify with UGS position display
; - Oscilloscope should show smooth velocity profile (no jerks at transitions)
; - Visual path should match image: rectangle → line → circle → line → rectangle
;
; CRITICAL TESTS:
; 1. Linear-to-arc transitions (4 corners + 2 circle entries)
; 2. Arc-to-linear transitions (4 corners + 2 circle exits)
; 3. Arc-to-arc transitions (circle split into two semicircles)
; 4. Subordinate axis coordination during diagonal moves (transition lines)
