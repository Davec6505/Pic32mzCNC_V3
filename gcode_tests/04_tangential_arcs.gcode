; Tangential Arcs Test - S-curve with smooth transitions
; Tests: Arc-to-arc transitions, tangency, continuous motion

G90                     ; Absolute positioning
G21                     ; Millimeters
G17                     ; XY plane
G92 X0 Y0 Z0           ; Set current position as origin

F350                   ; Smooth feedrate for curves

; S-Curve Pattern - Two tangential arcs forming smooth transition

; Start position
G0 X0 Y10              ; Start at left, middle height

; First arc - Clockwise (curves downward then right)
G2 X20 Y10 I10 J-10    ; CW arc: start (0,10) → end (20,10)
                       ; Center at (10,0)
                       ; Radius = sqrt(10^2 + 10^2) = 14.14mm
                       ; Creates right-leaning semicircular curve

; Second arc - Counter-clockwise (curves upward then right)
; Tangent to first arc at point (20,10)
G3 X40 Y10 I10 J10     ; CCW arc: start (20,10) → end (40,10)
                       ; Center at (30,20)
                       ; Radius = sqrt(10^2 + 10^2) = 14.14mm
                       ; Creates smooth S-curve transition

; Visual Result: Smooth S-shaped curve from (0,10) to (40,10)

; Alternative pattern - Figure-8 style with tangential transition
G0 X0 Y30              ; Move to second pattern start

; Upper circle (CCW)
G3 X0 Y50 I0 J10       ; Full circle: radius = 10mm, center at (0,40)

; At (0,50), tangentially transition to lower circle
; Lower circle (CW) - tangent at (0,50)
G2 X0 Y30 I0 J-10      ; Full circle: radius = 10mm, center at (0,40)
                       ; Creates figure-8 pattern

; Return to origin
G0 X0 Y0               ; Rapid home
