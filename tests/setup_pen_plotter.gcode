; ===== PEN PLOTTER SETUP SCRIPT =====
; Run this ONCE after $H (homing), before running any drawing files
;
; This establishes the work coordinate system:
;   Work Z=0   → Safe height (pen clear of paper)
;   Work Z=-60 → Paper contact (drawing position)
;
; Hardware: Z-axis homes to MAX (top) at ~70mm machine position
; Settings: $23=4 (Z homes to MAX), $132=72 (72mm Z travel)

G90             ; Absolute positioning mode
G1 Z12 F300     ; Jog down from homed position to safe height
                ; From machine Z=70 down to Z=12 (60mm clearance to paper)

G92 X0 Y0 Z0    ; Set current position as work coordinate zero
                ; Machine Z=12 becomes Work Z=0 (safe height)
                ; Now 60mm of travel available below this point

; ===== SETUP COMPLETE =====
; Machine is now at Work (0, 0, 0)
; Ready to run drawing files!
;
; To verify:
; Send ? to check position
; Should show: WPos:0.000,0.000,0.000
