

##How to Run Code: 

All of the code is contained in the main.c file. Timers 

##Components: 

Servo Motors – MG90D used to control the pitch and yaw of the robot to target the ping pong ball. We used PWM on two timers to control the full range of 90 degree motion the servos have. The servo control works by sending a 50Hz signal and varying the duty cycle to send between 1ms to 2ms pulses (1ms 0 degrees, 2ms 90 degrees).

DC Motors – used to power flywheel which launches the ball.

Atmega2560 – Elegoo mega board allows us to use four 16 bit timers and two 8 bit timers.

Ultrasonic – calculates distance between launcher and target as input to launch calculations.

Joystick – ADC input controls the axes of the launcher.

Launcher Body- 3d printed to house the components.


https://devpost.com/software/group-16-qth60u
