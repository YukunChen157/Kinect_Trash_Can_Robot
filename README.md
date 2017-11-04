Code for movement of robot

OPERATION INSTRUCTION:
1. Put the robot on the ground, turn its heading to where you want the robot goes as 0 degree. The heading direction is the center of the robot pointing to wheel No.1 (the opposite direction of the Arduino board).
2. Turn on the switches for all 3 motors, where the third one from the left also control the power of Arduino board.
3. Search for bluetooth BC-06 and pair it. Pearing code is 1234
4. Open TeraTerm, or any port signal editting APP, you shall see the greeting from the robot.
5. Press any key to start the calibration of the gyroscope, it will take about 20 secs to seattle it down. During this time, do not move the robot. Once seattled, the current heading offset of the robot will be displayed. During the whole movement of the robot, it will keep its head pointing to the same direction.
6. Enter the 7-digit movement signal to the robot. The first 3 digits are the angle information. It is the clockwise angle you want the robot moving to compare to the heading center (mentioned as previous), ranging from 0 to 359. The last four digits are the distance informatoin. It is the distance you want the robot to move in millimeter. Currently, due to the speed of the robot, it can only range from 0 to 1000 (At most 1 meter).
7. Once you enter the 7-digit signal, press space (' ') to send it and you will see the robot movement. If the signal is incorrect, the robot won't have any movement and the signal will be cleared.

APPROACH OF THE CODE:
1. Setup the motor and calibrate the gyroscope: setup() & motorsetUp();
2. In the mainloop, it will do the following:
	a. read the signal from the bluetooth and try to find the distance and angle when there is an ending key (space)
	b. when the angle and distance is captured, find out the runtime of the robot: getRunningTime()
	c. calculate the motors' parameter for the robot heading to certain angle
	d. get the gyroscope data, if the head of the robot is not pointing to the center, modify the motors' parameter to rectify it.
	e. send the motors' parameter to motor
	f. run the code c to e repeatly, until the runtime calculated in step b is approached.
	g. Once the runtime is approached, set the movement of 3 motors into 0 and start again from code a

NOTE:
1. The preset motor parameters, including parameter for motor1, 2, 3 heading to 0, 30,...,330 degree, and the runtime of the motor, are all based on testing. The result of the robot movement is not so accurate, it depends on battery level, slippery of the carpet etc.
2. Sometime the code will stop working in the middle, this is because the data from the gyroscope exceed the bandwidth. This event happens randomly, the solution is just restarting the whole process from the beginning.
