L + R + D + B: auton maker on/off





###### **when on:**



**U/D:** toggle between auton programs

**R:** create (overwrite) txt file using auton name, set to write



###### **loop:**



**U:** toggle movement type (P, T, D, M)

&nbsp;	P: drive to point movement (wait for confirm, wait for confirm, record last, write to file)

&nbsp;	T: turn movement (same format as DTP)

&nbsp;	D: dist. movement (wait for confirm, wait for confirm (allows for R press), record distance, write to file)

 	M: drive distance movement(wait for confirm, record first, wait for confirm, record last, calc. distance, write to file)

**L:** add/remove motion chain to current movement (disabled outside record period)

**D:** toggle timeout between 3000ms, 1000ms, 700ms, 200ms

**R:** Confirm



(general structure: 1 - in movement selector, wait for confirm, 2 - collect 1st data, wait for condirm, 3 - collect 2nd data, write to file



**Y:** toggle max voltage between 12V, 10V, 8V, 6V

**B:** (dist. movement only) toggle between front/back sensor (back sensor will use -1 modifier for dir)

**B:** (DTP, Drive Dist. only) toggle between forward/backward movement

**X, A:** MANUAL CONTROl



**L1, L2, R1, R2:** MANUAL CONTROL







###### **on auton run:**



open txt file using auton name, set to read, read by format below







###### **txt file format:**



"(movement) (modifiers) (timeout) (motionchain) (maxvolt)\\n"



(movement): P, T, D, M (DTP, Turn, Dist., Drive Dist. respectively)



(modifiers):

P - x, y, direction

T - angle

D - distance, sensor

M - distance, direction







###### **controller print format:**



(movement)

V: (voltage) ("S"/"D"): (sensor/direction)

T: (timeout) M: (motionchain)

