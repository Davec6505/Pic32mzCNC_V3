; Rectangle-to-Circle Handoff Test - PEN PLOTTER VERSION
; 
; Fully automatic - includes homing and setup
; No manual intervention required
;
; Origin: Lower-left of rectangle
; Rectangle: 20mm × 30mm with 3mm radius corners
; Circle: 20mm diameter (10mm radius) centered at (30, 15)

; =====================================
; AUTOMATIC SETUP
; =====================================


; Set this position as work coordinate zero
G92 X0 Y0 Z0

; Initialize
G21           ; Millimeters
G90           ; Absolute positioning
G94           ; Units per minute feedrate mode

; Work coordinate system now established:
;   Z=0    → Safe height (pen clear of paper)
;   Z=-60  → Pen touching paper (drawing position)
; Jog down to safe height above paper (12mm remaining travel = 60mm from top)
G90
G1 Z-50 F300
; =====================================
; MOVE TO START POSITION
; =====================================

; Pen is at safe height (Z=0 work coordinates)
; Move to rectangle start position
G0 X3 Y0 F1000

; Lower pen DOWN to paper (60mm down from work zero)
; With $132=72 and work zero at 12mm remaining travel, this goes to -48mm work / 24mm machine
G1 Z-73 F300

; =====================================
; RECTANGLE WITH ROUNDED CORNERS
; =====================================

G1 X17 Y0 F1000
G2 X20 Y3 I0 J3
G1 X20 Y27
G2 X17 Y30 I-3 J0
G1 X3 Y30
G2 X0 Y27 I0 J-3
G1 X0 Y3
G2 X3 Y0 I3 J0

; =====================================
; TRANSITION: RECTANGLE TO CIRCLE
; =====================================

; Lift pen UP to safe height
G1 Z-50 F300

; Move to circle start position
G1 X40 Y15 F1000

; Lower pen DOWN onto paper
G1 Z-73 F300

; =====================================
; CIRCULAR PATH
; =====================================

G2 X20 Y15 I-10 J0 F1000
G2 X40 Y15 I10 J0

; =====================================
; RETURN TO ORIGIN
; =====================================

; Lift pen UP
G1 Z-50 F300

; Return to origin
G1 X0 Y0 F1000

G1 Z0 F300

; Final position: (0, 0, 0) - at safe height

M30