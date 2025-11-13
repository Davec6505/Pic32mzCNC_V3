; Simple Arc Test - No G4 dwell commands
G21
G90
G17
G94
M3 S1000
G0 Z5 F1500
G0 X0 Y0
G0 Z0
G2 X10 Y0 I5 J0 F1000    ; ← Corrected: I5 instead of I10
G0 Z5
G0 X0 Y0
M5
