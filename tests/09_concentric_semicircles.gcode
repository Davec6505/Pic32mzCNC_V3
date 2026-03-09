; 09_concentric_semicircles.gcode
; Concentric CW/CCW semicircle pairs, radii 60->10mm (step 10mm)
; Centre: (70,70).  Each pair: G0 rapid to left tangent → G2 (CW) → G3 (CCW)
; Feedrate F1500 mm/min
; Expected finish at MPos:70,70 (within ~0.05mm)
; Run time ~55-60 s
G21 G90 G17
G0 X70 Y70 F3000
G92 X70 Y70

G0 X10 Y70
F1500
G2 X130 Y70 I60 J0
G3 X10  Y70 I-60 J0

G0 X20 Y70
G2 X120 Y70 I50 J0
G3 X20  Y70 I-50 J0

G0 X30 Y70
G2 X110 Y70 I40 J0
G3 X30  Y70 I-40 J0

G0 X40 Y70
G2 X100 Y70 I30 J0
G3 X40  Y70 I-30 J0

G0 X50 Y70
G2 X90 Y70 I20 J0
G3 X50 Y70 I-20 J0

G0 X60 Y70
G2 X80 Y70 I10 J0
G3 X60 Y70 I-10 J0

G0 X70 Y70
M5
M2
