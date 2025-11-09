; Simple Square Test - 20mm x 20mm, repeated twice
; Tests: G90/G91, G0/G1, F command, position tracking

G90                     ; Absolute positioning
G21                     ; Millimeters
G92 X0 Y0 Z0           ; Set current position as origin

; First Square
G0 X0 Y0               ; Rapid to start position
F500                   ; Set feedrate to 500 mm/min
G1 X20 Y0              ; Line to right
G1 X20 Y20             ; Line up
G1 X0 Y20              ; Line to left
G1 X0 Y0               ; Line back to origin

; Second Square (offset 30mm in X)
G0 X30 Y0              ; Rapid to second start position
G1 X50 Y0              ; Line to right
G1 X50 Y20             ; Line up
G1 X30 Y20             ; Line to left
G1 X30 Y0              ; Line back to start

; Return to origin
G0 X0 Y0               ; Rapid home
