; Single Semicircle Test - Valid geometry
; Tests basic arc interpolation without invalid commands

G90                     ; Absolute positioning
G21                     ; Millimeters
G17                     ; XY plane

; Move to start position
G0 X0 Y0 Z0            ; Start at origin

; Single semicircle arc
G2 X10 Y0 I5 J0 F1000  ; CW semicircle from (0,0) to (10,0), center at (5,0), radius=5mm

; Return home
G0 X0 Y0               ; Back to origin
