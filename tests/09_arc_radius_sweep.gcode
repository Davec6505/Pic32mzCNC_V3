; 09_arc_radius_sweep.gcode
; Arc radius sweep: CW/CCW semicircle pairs at R=60,50,40,30,20,10mm
;
; Purpose: Validate arc geometry across multiple radii.
;          Each group: G2 (CW) then G3 (CCW) semicircle, identical geometry.
;          Radii step down from 60mm to 10mm.  G0 repositions between groups.
;
; Arc geometry (XY plane, arc centre at work (70,70)):
;   CW  (G2): start=(70-R, 70) -> end=(70+R, 70),  I=+R  J=0
;   CCW (G3): start=(70+R, 70) -> end=(70-R, 70),  I=-R  J=0
;
; All coordinates positive — machine X stays in range 10..130 mm.
; Assumes machine starts near home (WCS offset ~0) before running.
; Required work area: at least 140 x 140 mm.

G21       ; metric
G90       ; absolute
G17       ; XY plane
G0 X70 Y70 F3000  ; move to arc centre (rapid)
G92 X70 Y70       ; declare current position as work (70,70) -> work = machine

; ============================================================
; R = 60 mm   machine X: 10..130   machine Y: 10..130
; ============================================================
G0 X10 Y70
F1500
G2 X130 Y70 I60  J0     ; CW  semicircle R=60
G3 X10  Y70 I-60 J0     ; CCW semicircle R=60

; ============================================================
; R = 50 mm   machine X: 20..120   machine Y: 20..120
; ============================================================
G0 X20 Y70
G2 X120 Y70 I50  J0
G3 X20  Y70 I-50 J0

; ============================================================
; R = 40 mm   machine X: 30..110   machine Y: 30..110
; ============================================================
G0 X30 Y70
G2 X110 Y70 I40  J0
G3 X30  Y70 I-40 J0

; ============================================================
; R = 30 mm   machine X: 40..100   machine Y: 40..100
; ============================================================
G0 X40 Y70
G2 X100 Y70 I30  J0
G3 X40  Y70 I-30 J0

; ============================================================
; R = 20 mm   machine X: 50..90    machine Y: 50..90
; ============================================================
G0 X50 Y70
G2 X90 Y70 I20  J0
G3 X50 Y70 I-20 J0

; ============================================================
; R = 10 mm   machine X: 60..80    machine Y: 60..80
; ============================================================
G0 X60 Y70
G2 X80 Y70 I10  J0
G3 X60 Y70 I-10 J0

; Return to centre
G0 X70 Y70
M5
M2
