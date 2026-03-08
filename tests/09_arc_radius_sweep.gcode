; 09_arc_radius_sweep.gcode
; Arc radius sweep: CW/CCW semicircle pairs at R=60,50,40,30,20,10mm
;
; Purpose: Determine if stall is radius-dependent (linear vs arc boundary).
;          Each step does exactly 2 arcs: G2 (CW) then G3 (CCW) semicircle.
;          Radii step down from 60mm to 10mm.  A rapid G0 repositions
;          between groups so each pair is geometrically identical.
;
; Arc geometry (all in XY plane, Y=0 axis):
;   CW  (G2): start=(-R,0)  ->  end=(+R,0),  center=(0,0),  I=+R  J=0
;   CCW (G3): start=(+R,0)  ->  end=(-R,0),  center=(0,0),  I=-R  J=0
;
; Set G92 X0 Y0 at a safe central position before running.
; Max X excursion: -60 .. +60 mm.

G21       ; metric
G90       ; absolute
G17       ; XY plane
G92 X0 Y0 ; work origin at current position

; ============================================================
; R = 60 mm
; ============================================================
G0 X-60 Y0 F3000
F1500
G2 X60  Y0 I60  J0     ; CW  semicircle R=60
G3 X-60 Y0 I-60 J0     ; CCW semicircle R=60

; ============================================================
; R = 50 mm
; ============================================================
G0 X-50 Y0
G2 X50  Y0 I50  J0
G3 X-50 Y0 I-50 J0

; ============================================================
; R = 40 mm
; ============================================================
G0 X-40 Y0
G2 X40  Y0 I40  J0
G3 X-40 Y0 I-40 J0

; ============================================================
; R = 30 mm
; ============================================================
G0 X-30 Y0
G2 X30  Y0 I30  J0
G3 X-30 Y0 I-30 J0

; ============================================================
; R = 20 mm
; ============================================================
G0 X-20 Y0
G2 X20  Y0 I20  J0
G3 X-20 Y0 I-20 J0

; ============================================================
; R = 10 mm
; ============================================================
G0 X-10 Y0
G2 X10  Y0 I10  J0
G3 X-10 Y0 I-10 J0

; Return home
G0 X0 Y0
M5
M2
