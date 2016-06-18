;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; all home
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
G28
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; set Z
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 5cm - first approx - adjust
G1 Z50
; 2cm - second approx - adjust
G1 Z20
; 1cm - third approx - adjust, e.g.
; G1 X15
G1 Z10
; z == 0 - main adjust, e.g.
; e.g. ; G1 X5
G1 Z0
;adjust Z, e.g.
; G1 Z-0.2
; G1 X1.5
;
;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; set X & Y
; now check the printing area
; the distance from the nozzle must be about the same
; the distance must be enough to push paper under the nozzle
; change Z if necessary
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; 
G1 X50
;
G1 Y50
;
G1 X0
G1 X-50
;
G1 Y0
G1 Y-50
;
G1 X0
G1 X50

G1 Y0
G1 Y50
;
G1 X0 Y0
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; all home
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
G28
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; now update firmware
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

