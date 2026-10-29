; A valid move followed by unsupported homing must reject the entire import.
G21
G90
M82
G92 E0
G1 X1 E0.1 F60
G28
G1 X2 E0.2
