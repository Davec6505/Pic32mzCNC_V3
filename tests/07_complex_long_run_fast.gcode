; ============================================================
; Complex Long-Running System Test
; File: tests/07_complex_long_run.gcode
; Tests: Linear moves, G2/G3 arcs, feedrate sweep, G4 dwell,
;        concentric rectangles, star pattern, zigzag raster,
;        diagonal raster, semicircle shuttle, high-speed sprint
; Work area: 60x60mm from current zero (bench-safe, no material needed)
; Estimated run time: 8-12 minutes at typical feedrates
; ============================================================

G21 G90 G17      ; mm mode, absolute positioning, XY plane
G92 X0 Y0 Z0     ; zero at current position

; ============================================================
; SECTION 1 - Feedrate ladder: rectangle at 4 speeds
; ============================================================
G1 F5000
G1 X60 Y0
G1 X60 Y60
G1 X0 Y60
G1 X0 Y0

G1 F6000
G1 X60 Y0
G1 X60 Y60
G1 X0 Y60
G1 X0 Y0

G1 F5500
G1 X60 Y0
G1 X60 Y60
G1 X0 Y60
G1 X0 Y0

G1 F6000
G1 X60 Y0
G1 X60 Y60
G1 X0 Y60
G1 X0 Y0

; ============================================================
; SECTION 2 - Concentric rectangles (inward spiral)
; ============================================================
G1 F4000
G1 X5 Y5
G1 X55 Y5
G1 X55 Y55
G1 X5 Y55
G1 X5 Y5

G1 X10 Y10
G1 X50 Y10
G1 X50 Y50
G1 X10 Y50
G1 X10 Y10

G1 X15 Y15
G1 X45 Y15
G1 X45 Y45
G1 X15 Y45
G1 X15 Y15

G1 X20 Y20
G1 X40 Y20
G1 X40 Y40
G1 X20 Y40
G1 X20 Y20

G1 X25 Y25
G1 X35 Y25
G1 X35 Y35
G1 X25 Y35
G1 X25 Y25

G1 X0 Y0

; ============================================================
; SECTION 3 - Star / diagonal cross pattern
; ============================================================
G1 F8000
G1 X30 Y0
G1 X30 Y60
G1 X30 Y30
G1 X0 Y30
G1 X60 Y30
G1 X30 Y30
G1 X0 Y0
G1 X60 Y60
G1 X30 Y30
G1 X60 Y0
G1 X0 Y60
G1 X0 Y0

; ============================================================
; SECTION 4 - Full CW circles (G2) - 3 radii
; Circle center: (30,30)
; ============================================================
G1 F8000
G1 X40 Y30        ; R=10 start point (east of center)
G2 X40 Y30 I-10 J0   ; CW full circle R=10

G1 X50 Y30        ; R=20 start point
G2 X50 Y30 I-20 J0   ; CW full circle R=20

G1 X30 Y0         ; R=30 start point (south of center)
G2 X30 Y0 I0 J30     ; CW full circle R=30

; ============================================================
; SECTION 5 - Full CCW circles (G3) - 2 radii
; ============================================================
G1 X40 Y30
G3 X40 Y30 I-10 J0   ; CCW full circle R=10

G1 X50 Y30
G3 X50 Y30 I-20 J0   ; CCW full circle R=20

G1 X0 Y0

; ============================================================
; SECTION 6 - Quarter arc chain: CCW then CW
; Circle center (30,30) R=20
; Landmarks: E=(50,30) N=(30,50) W=(10,30) S=(30,10)
; ============================================================
G1 F6000
G1 X50 Y30         ; start east

; CCW quarter arcs: E->N->W->S->E
G3 X30 Y50 I-20 J0    ; 0deg -> 90deg
G3 X10 Y30 I0 J-20    ; 90deg -> 180deg
G3 X30 Y10 I20 J0     ; 180deg -> 270deg
G3 X50 Y30 I0 J20     ; 270deg -> 360deg

; CW quarter arcs back: E->S->W->N->E
G2 X30 Y10 I-20 J0    ; 0deg -> -90deg (270)
G2 X10 Y30 I0 J20     ; 270deg -> 180deg
G2 X30 Y50 I20 J0     ; 180deg -> 90deg
G2 X50 Y30 I0 J-20    ; 90deg -> 0deg

G1 X0 Y0

; ============================================================
; SECTION 7 - Zigzag raster (horizontal)
; ============================================================
G1 F5500
G1 X60 Y0
G1 X0 Y6
G1 X60 Y12
G1 X0 Y18
G1 X60 Y24
G1 X0 Y30
G1 X60 Y36
G1 X0 Y42
G1 X60 Y48
G1 X0 Y54
G1 X60 Y60
G1 X0 Y0

; ============================================================
; SECTION 8 - Diagonal raster (vertical sweep)
; ============================================================
G1 F4000
G1 X0 Y60
G1 X6 Y0
G1 X12 Y60
G1 X18 Y0
G1 X24 Y60
G1 X30 Y0
G1 X36 Y60
G1 X42 Y0
G1 X48 Y60
G1 X54 Y0
G1 X60 Y60
G1 X0 Y0

; ============================================================
; SECTION 9 - Semicircle shuttle stress test
; Center (30,30) R=30: W=(0,30) E=(60,30)
; G2 forward = top semicircle, G2 back = bottom semicircle
; ============================================================
G1 F1200
G1 X0 Y30
G2 X60 Y30 I30 J0     ; CW top semicircle
G2 X0 Y30 I-30 J0     ; CW bottom semicircle
G3 X60 Y30 I30 J0     ; CCW bottom semicircle
G3 X0 Y30 I-30 J0     ; CCW top semicircle
G2 X60 Y30 I30 J0
G2 X0 Y30 I-30 J0
G3 X60 Y30 I30 J0
G3 X0 Y30 I-30 J0

G1 X0 Y0

; ============================================================
; SECTION 10 - Dwell at corners (G4)
; ============================================================
G1 F4000
G1 X0 Y0
G4 P1.0
G1 X60 Y0
G4 P1.0
G1 X60 Y60
G4 P1.0
G1 X0 Y60
G4 P1.0
G1 X0 Y0
G4 P2.0

; ============================================================
; SECTION 11 - High-speed sprint
; ============================================================
G1 F5000
G1 X60 Y0
G1 X60 Y60
G1 X0 Y60
G1 X0 Y0
G1 X60 Y60
G1 X0 Y0
G1 X60 Y0
G1 X0 Y60
G1 X30 Y30
G1 X0 Y0

; ============================================================
; END - Return to origin with final dwell
; ============================================================
G4 P0.5
G1 F3000
G1 X0 Y0
