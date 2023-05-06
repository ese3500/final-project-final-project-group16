## Pong Artillery: 
The Pong Artillery (or Beer Bombadier) is a ping pong ball launcher made to simulate the technology behind artillery cannons. It was created by Olivier Pham and Anish Agrawal for the ESE 3500: Embedded Systems Final Project. Our goal was to create a launcher that was fully controllable manually and that could automatically aim a shot at a target given a distance from an ultrasonic reading.

## How to Run Code: 

All of the code is contained in the main.c file. Timers 1 and 5 control the servos by changing the OCR values. Changing the OCR3A value under motor_init() will change the pwm speed of the motors (255 being the max and 1 being the min). Timer 4 is used to manage the ultrasonic sensor interrupts. There is distance averaging built into the project that stores 10 distance values at a time and takes the average of them to eliminate noise. The getAngle function returns the OCR value (angle of projectile) associated with a given distance. It uses projectile motion calcs to get an angle from the required range given by the ultrasonic sensor. The initial distance value is the maximum range of the launcher at 45 degrees. Changing this will change the launch velocity used in the calculations.

## Components: 

Servo Motors – MG90D used to control the pitch and yaw of the robot to target the ping pong ball. We used PWM on two timers to control the full range of 90 degree motion the servos have. The servo control works by sending a 50Hz signal and varying the duty cycle to send between 1ms to 2ms pulses (1ms 0 degrees, 2ms 90 degrees).

DC Motors – Used to power flywheel which launches the ball.

Atmega2560 – Elegoo mega board allows us to use four 16 bit timers and two 8 bit timers.

Ultrasonic – Calculates distance between launcher and target as input to launch calculations.

Joystick – ADC input controls the axes of the launcher.

Launcher Body- 3d printed to house the components.

### Here is the devpost link for more information:
https://devpost.com/software/group-16-qth60u
