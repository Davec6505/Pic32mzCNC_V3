; Simple 3-Arc Test - No Dwells
; Test if UGS can stream 3 consecutive arcs

G21 G90 G17 G94
M3 S1000

G0 Z5 F1500
G0 X0 Y0
G0 Z0

; Arc 1: Quarter circle CW
G2 X10 Y0 I5 J0 F1000

; Arc 2: Quarter circle CCW  
G3 X10 Y10 I0 J5 F1000

; Arc 3: Semicircle CW
G2 X0 Y10 I-5 J0 F1000

G0 Z5
G0 X0 Y0
M5
