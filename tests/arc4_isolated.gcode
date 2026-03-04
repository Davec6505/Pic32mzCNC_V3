; Isolated arc4 test - runs only arc4 so we can see ARC_SETUP debug output
; without UART TX buffer overflow from arcs 1-3
; Arc4 = G2X20Y10I10J0F500
; Expected: start=(0,10), center=(10,10), end=(20,10), CW semicircle
G21
G90
G17
G0 X0 Y0 Z0
G0 X0 Y10
G2 X20 Y10 I10 J0 F200
G0 Z5
G0 X0 Y0
