; Rounded Rectangle Test - 40mm x 20mm with 5mm radius corners
; Tests: Continuous motion, G1/G2 transitions, tangential arcs

G90                     ; Absolute positioning
G21                     ; Millimeters
G17                     ; XY plane
G92 X0 Y0 Z0           ; Set current position as origin

F400                   ; Set feedrate to 400 mm/min

; Start at bottom-left, offset by corner radius
G0 X5 Y0               ; Rapid to start position

; Bottom edge
G1 X35 Y0              ; Line to bottom-right (before corner)

; Bottom-right corner (radius = 5mm)
G2 X40 Y5 I0 J5        ; CW arc to right edge

; Right edge
G1 X40 Y15             ; Line up right side

; Top-right corner
G2 X35 Y20 I-5 J0      ; CW arc to top edge

; Top edge
G1 X5 Y20              ; Line to top-left (before corner)

; Top-left corner
G2 X0 Y15 I0 J-5       ; CW arc to left edge

; Left edge
G1 X0 Y5               ; Line down left side

; Bottom-left corner (close the shape)
G2 X5 Y0 I5 J0         ; CW arc back to start

; Return to origin
G0 X0 Y0               ; Rapid home
