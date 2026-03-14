; =============================================================================
; LASER UTILITY — PIC32MZ CNC Firmware  v1.0  (2026-03-14)
; GRBL v1.1 Laser Mode — Dynamic Power Scaling via S-curve ISR
;
; Prerequisites — send once before running this file:
;   $32=1         laser mode ON  (power tracks feedrate automatically)
;   $30=1000      S scale: S1000 = 100 % power
;   $31=0         minimum power = 0 (laser off at S0)
;   $110=5000     X max rate mm/min
;   $111=5000     Y max rate mm/min
;   $120=1000     X acceleration mm/s^2
;   $121=1000     Y acceleration mm/s^2
;
; Work origin: lower-left corner of test area (set with G92 or fixture).
; Assumed test area: 120 mm (X) × 100 mm (Y).
; Spot size / kerf will vary with optics — adjust S and F to taste.
;
; Sections:
;   A. Power calibration  — 10 horizontal lines at increasing S (same F)
;   B. Speed calibration  — 5 horizontal lines at increasing F (same S)
;   C. Corner sharpness   — nested squares testing how well power dims at corners
;   D. Continuous arc     — full circle to verify arc-speed power continuity
;   E. Spiral             — tightening squares to test junction blending
;   F. Raster fill        — dense hatch to engrave a solid rectangle
;   G. Return             — go home, laser off
; =============================================================================

G21 G90 G94         ; mm, absolute, per-minute feed mode
M5                  ; laser OFF (safety)
G92 X0 Y0 Z0        ; set work origin at current position
G0 X0 Y0            ; ensure we are at origin

; =============================================================================
; A. POWER CALIBRATION  (fixed F3000, varying S)
;    Each line is 80 mm long, spaced 5 mm apart.
;    Compare burn width/depth to choose working S value.
; =============================================================================

G0 X0 Y0
M3 S100             ; 10 % power
G1 X80 F3000
M5
G0 X0 Y5

M3 S200             ; 20 %
G1 X80 F3000
M5
G0 X0 Y10

M3 S350             ; 35 %
G1 X80 F3000
M5
G0 X0 Y15

M3 S500             ; 50 %
G1 X80 F3000
M5
G0 X0 Y20

M3 S650             ; 65 %
G1 X80 F3000
M5
G0 X0 Y25

M3 S800             ; 80 %
G1 X80 F3000
M5
G0 X0 Y30

M3 S900             ; 90 %
G1 X80 F3000
M5
G0 X0 Y35

M3 S1000            ; 100 %
G1 X80 F3000
M5
G0 X0 Y40

; =============================================================================
; B. SPEED CALIBRATION  (fixed S700, varying F)
;    Same laser power, different travel speed.
;    Faster = lighter mark.  Slower = deeper cut.
; =============================================================================

G0 X0 Y50
M3 S700
G1 X80 F1000        ; 1000 mm/min (slow, dark)
M5
G0 X0 Y55

M3 S700
G1 X80 F2000
M5
G0 X0 Y60

M3 S700
G1 X80 F3000        ; nominal working speed
M5
G0 X0 Y65

M3 S700
G1 X80 F5000
M5
G0 X0 Y70

M3 S700
G1 X80 F8000        ; fast, light mark
M5
G0 X0 Y75

; =============================================================================
; C. CORNER SHARPNESS  — nested squares
;    Dynamic power scaling ($32=1) should dim the laser at each corner
;    where the machine decelerates, preventing over-burn.
;    Compare inner corners with outer corners to verify.
; =============================================================================

G0 X90 Y0
M3 S800

; Outer square 30 × 30 mm, F4000
G1 X120 Y0   F4000
G1 X120 Y30  F4000
G1 X90  Y30  F4000
G1 X90  Y0   F4000
M5

G0 X93 Y3
M3 S800

; Middle square 24 × 24 mm, faster (tighter corner test)
G1 X117 Y3   F6000
G1 X117 Y27  F6000
G1 X93  Y27  F6000
G1 X93  Y3   F6000
M5

G0 X96 Y6
M3 S800

; Inner square 18 × 18 mm, fastest
G1 X114 Y6   F8000
G1 X114 Y24  F8000
G1 X96  Y24  F8000
G1 X96  Y6   F8000
M5

; =============================================================================
; D. CONTINUOUS ARC  — full circle, R=15 mm
;    Verifies that arc chord stitching does not cause power flicker.
;    Power should be constant during the arc if feedrate matches nominal.
;    Split into two 180° arcs because G2/G3 with I=J=0 is undefined.
; =============================================================================

G0 X35 Y85           ; arc centre will be at (50, 85), start at (35, 85)
M3 S700
G2 X65 Y85 I15 J0 F3000   ; CW semicircle, R=15: (35,85) → (65,85)
G2 X35 Y85 I-15 J0 F3000  ; CW semicircle, closes circle
M5

; =============================================================================
; E. SPIRAL (tightening squares) — tests junction blending under power scaling
; =============================================================================

G0 X0 Y0
M3 S600

G1 X40 Y0   F5000
G1 X40 Y40  F5000
G1 X0  Y40  F5000
G1 X0  Y5   F5000    ; inward
G1 X35 Y5   F5000
G1 X35 Y35  F5000
G1 X5  Y35  F5000
G1 X5  Y10  F5000
G1 X30 Y10  F5000
G1 X30 Y30  F5000
G1 X10 Y30  F5000
G1 X10 Y15  F5000
G1 X25 Y15  F5000
G1 X25 Y25  F5000
G1 X15 Y25  F5000
G1 X15 Y20  F5000
G1 X20 Y20  F5000

M5

; =============================================================================
; F. RASTER FILL  — dense horizontal hatch over a 40 × 15 mm rectangle
;    Line pitch: 0.15 mm (adjust to laser spot size).
;    Demonstrates that the power follows feedrate smoothly on short strokes.
; =============================================================================

G0 X0 Y0

; Macro-style: 100 hatch lines; line spacing 0.15 mm.
; Y range: 0 to 15 mm  →  ~100 passes.
; (Expand or reduce count to suit material.)

M3 S800
G1 X40 Y0    F2000
G0 X0  Y0.15
G1 X40 Y0.15 F2000
G0 X0  Y0.30
G1 X40 Y0.30 F2000
G0 X0  Y0.45
G1 X40 Y0.45 F2000
G0 X0  Y0.60
G1 X40 Y0.60 F2000
G0 X0  Y0.75
G1 X40 Y0.75 F2000
G0 X0  Y0.90
G1 X40 Y0.90 F2000
G0 X0  Y1.05
G1 X40 Y1.05 F2000
G0 X0  Y1.20
G1 X40 Y1.20 F2000
G0 X0  Y1.35
G1 X40 Y1.35 F2000
G0 X0  Y1.50
G1 X40 Y1.50 F2000
G0 X0  Y1.65
G1 X40 Y1.65 F2000
G0 X0  Y1.80
G1 X40 Y1.80 F2000
G0 X0  Y1.95
G1 X40 Y1.95 F2000
G0 X0  Y2.10
G1 X40 Y2.10 F2000
G0 X0  Y2.25
G1 X40 Y2.25 F2000
G0 X0  Y2.40
G1 X40 Y2.40 F2000
G0 X0  Y2.55
G1 X40 Y2.55 F2000
G0 X0  Y2.70
G1 X40 Y2.70 F2000
G0 X0  Y2.85
G1 X40 Y2.85 F2000
G0 X0  Y3.00
G1 X40 Y3.00 F2000
G0 X0  Y3.15
G1 X40 Y3.15 F2000
G0 X0  Y3.30
G1 X40 Y3.30 F2000
G0 X0  Y3.45
G1 X40 Y3.45 F2000
G0 X0  Y3.60
G1 X40 Y3.60 F2000
G0 X0  Y3.75
G1 X40 Y3.75 F2000
G0 X0  Y3.90
G1 X40 Y3.90 F2000
G0 X0  Y4.05
G1 X40 Y4.05 F2000
G0 X0  Y4.20
G1 X40 Y4.20 F2000
G0 X0  Y4.35
G1 X40 Y4.35 F2000
G0 X0  Y4.50
G1 X40 Y4.50 F2000
G0 X0  Y4.65
G1 X40 Y4.65 F2000
G0 X0  Y4.80
G1 X40 Y4.80 F2000
G0 X0  Y4.95
G1 X40 Y4.95 F2000
G0 X0  Y5.10
G1 X40 Y5.10 F2000
G0 X0  Y5.25
G1 X40 Y5.25 F2000
G0 X0  Y5.40
G1 X40 Y5.40 F2000
G0 X0  Y5.55
G1 X40 Y5.55 F2000
G0 X0  Y5.70
G1 X40 Y5.70 F2000
G0 X0  Y5.85
G1 X40 Y5.85 F2000
G0 X0  Y6.00
G1 X40 Y6.00 F2000
G0 X0  Y6.15
G1 X40 Y6.15 F2000
G0 X0  Y6.30
G1 X40 Y6.30 F2000
G0 X0  Y6.45
G1 X40 Y6.45 F2000
G0 X0  Y6.60
G1 X40 Y6.60 F2000
G0 X0  Y6.75
G1 X40 Y6.75 F2000
G0 X0  Y6.90
G1 X40 Y6.90 F2000
G0 X0  Y7.05
G1 X40 Y7.05 F2000
G0 X0  Y7.20
G1 X40 Y7.20 F2000
G0 X0  Y7.35
G1 X40 Y7.35 F2000
G0 X0  Y7.50
G1 X40 Y7.50 F2000
G0 X0  Y7.65
G1 X40 Y7.65 F2000
G0 X0  Y7.80
G1 X40 Y7.80 F2000
G0 X0  Y7.95
G1 X40 Y7.95 F2000
G0 X0  Y8.10
G1 X40 Y8.10 F2000
G0 X0  Y8.25
G1 X40 Y8.25 F2000
G0 X0  Y8.40
G1 X40 Y8.40 F2000
G0 X0  Y8.55
G1 X40 Y8.55 F2000
G0 X0  Y8.70
G1 X40 Y8.70 F2000
G0 X0  Y8.85
G1 X40 Y8.85 F2000
G0 X0  Y9.00
G1 X40 Y9.00 F2000
G0 X0  Y9.15
G1 X40 Y9.15 F2000
G0 X0  Y9.30
G1 X40 Y9.30 F2000
G0 X0  Y9.45
G1 X40 Y9.45 F2000
G0 X0  Y9.60
G1 X40 Y9.60 F2000
G0 X0  Y9.75
G1 X40 Y9.75 F2000
G0 X0  Y9.90
G1 X40 Y9.90 F2000
G0 X0  Y10.05
G1 X40 Y10.05 F2000
G0 X0  Y10.20
G1 X40 Y10.20 F2000
G0 X0  Y10.35
G1 X40 Y10.35 F2000
G0 X0  Y10.50
G1 X40 Y10.50 F2000
G0 X0  Y10.65
G1 X40 Y10.65 F2000
G0 X0  Y10.80
G1 X40 Y10.80 F2000
G0 X0  Y10.95
G1 X40 Y10.95 F2000
G0 X0  Y11.10
G1 X40 Y11.10 F2000
G0 X0  Y11.25
G1 X40 Y11.25 F2000
G0 X0  Y11.40
G1 X40 Y11.40 F2000
G0 X0  Y11.55
G1 X40 Y11.55 F2000
G0 X0  Y11.70
G1 X40 Y11.70 F2000
G0 X0  Y11.85
G1 X40 Y11.85 F2000
G0 X0  Y12.00
G1 X40 Y12.00 F2000
G0 X0  Y12.15
G1 X40 Y12.15 F2000
G0 X0  Y12.30
G1 X40 Y12.30 F2000
G0 X0  Y12.45
G1 X40 Y12.45 F2000
G0 X0  Y12.60
G1 X40 Y12.60 F2000
G0 X0  Y12.75
G1 X40 Y12.75 F2000
G0 X0  Y12.90
G1 X40 Y12.90 F2000
G0 X0  Y13.05
G1 X40 Y13.05 F2000
G0 X0  Y13.20
G1 X40 Y13.20 F2000
G0 X0  Y13.35
G1 X40 Y13.35 F2000
G0 X0  Y13.50
G1 X40 Y13.50 F2000
G0 X0  Y13.65
G1 X40 Y13.65 F2000
G0 X0  Y13.80
G1 X40 Y13.80 F2000
G0 X0  Y13.95
G1 X40 Y13.95 F2000
G0 X0  Y14.10
G1 X40 Y14.10 F2000
G0 X0  Y14.25
G1 X40 Y14.25 F2000
G0 X0  Y14.40
G1 X40 Y14.40 F2000
G0 X0  Y14.55
G1 X40 Y14.55 F2000
G0 X0  Y14.70
G1 X40 Y14.70 F2000
G0 X0  Y14.85
G1 X40 Y14.85 F2000
G0 X0  Y15.00
G1 X40 Y15.00 F2000

M5

; =============================================================================
; G. DONE — return to home, confirm laser off
; =============================================================================

G0 X0 Y0
M5              ; laser OFF (redundant safety)
M30             ; program end
