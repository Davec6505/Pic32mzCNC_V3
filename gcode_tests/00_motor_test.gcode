; Motor Movement Debug Test
; Tests each axis individually at slow speed
; Watch for ANY motor movement, even slight vibration

G90                     ; Absolute positioning
G21                     ; Millimeters
G92 X0 Y0 Z0 A0        ; Set origin

; Test X axis - 10mm forward, then back
F100                    ; Very slow - 100mm/min
G1 X10                  ; Move +10mm in X
G4 P1000                ; Wait 1 second
G1 X0                   ; Return to origin
G4 P1000

; Test Y axis - 10mm forward, then back
G1 Y10                  ; Move +10mm in Y
G4 P1000
G1 Y0                   ; Return to origin
G4 P1000

; Test Z axis - 5mm up, then down
G1 Z5                   ; Move +5mm in Z
G4 P1000
G1 Z0                   ; Return to origin
G4 P1000

; Faster test - does speed help?
F500                    ; 500mm/min
G1 X20                  ; Faster move
G4 P500
G1 X0
G4 P500

; Very slow test - is it TOO slow?
F50                     ; Very slow - 50mm/min
G1 X5
G4 P1000
G1 X0

; End
G92 X0 Y0 Z0           ; Reset origin
