; Rectangle-to-Circle Handoff Test
;
; Rectangle: 20mm x 30mm with 3mm radius corners, origin at lower-left
; Circle:    10mm radius, centred at X30 Y15
; Drawing feedrate: F300
; Pen down: Z73   Pen up: Z-50

G21           ; Millimeters
G90           ; Absolute positioning
G94           ; Units per minute feedrate mode
G92 X0 Y0 Z0  ; Set work zero here

; ── Safe height ──────────────────────────────────────────────────────────────
G1 Z10 F2300

; ── Rectangle with rounded corners ──────────────────────────────────────────
G1 X3 Y0 F300
G1 Z13 F2300          ; Pen down

G1 X17 Y0 F300
G2 X20 Y3 I0 J3 F300
G1 X20 Y27 F300
G2 X17 Y30 I-3 J0 F300
G1 X3 Y30 F300
G2 X0 Y27 I0 J-3 F300
G1 X0 Y3 F300
G2 X3 Y0 I3 J0 F300

G1 Z0 F2300         ; Pen up

; ── Full circle (two semicircles CW, centre X30 Y15, radius 10mm) ────────────
G0 X40 Y15
G1 Z13 F1300          ; Pen down

G2 X20 Y15 I-10 J0 F300
G2 X40 Y15 I10 J0 F300

G1 Z0 F2300         ; Pen up

; ── Return ───────────────────────────────────────────────────────────────────
G0 X0 Y0


; Return to origin
G1 X0 Y0 F500

G1 Z2 F1300

; Final position: (0, 0, 0) - at safe height

M30