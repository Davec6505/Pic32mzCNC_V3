; Simple Arc Test - Quarter circle and semicircle
; Tests: G2/G3, I/J parameters, arc interpolation

G90                     ; Absolute positioning
G21                     ; Millimeters
G17                     ; XY plane
G92 X0 Y0 Z0           ; Set current position as origin

; Quarter circle - Clockwise (G2)
G0 X0 Y0               ; Start at origin
F300                   ; Slower feedrate for arcs
G2 X10 Y10 I10 J0      ; CW arc: end at (10,10), center offset I=10, J=0
                       ; Creates quarter circle, radius = 10mm

; Line to next start position
G1 X20 Y10             ; Move right

; Quarter circle - Counter-clockwise (G3)
G3 X30 Y0 I0 J-10      ; CCW arc: end at (30,0), center offset I=0, J=-10
                       ; Creates quarter circle, radius = 10mm

; Semicircle test
G0 X40 Y0              ; Move to semicircle start
G2 X60 Y0 I10 J0       ; CW semicircle: end at (60,0), center at (50,0)
                       ; Creates semicircle, radius = 10mm

; Return to origin
G0 X0 Y0               ; Rapid home
