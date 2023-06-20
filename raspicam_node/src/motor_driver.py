#-----------------------------------------------------> HARDWARE CONNECTIONS <-----------------------------------------------------------

#  LEFT_MOTOR_CTL   RIGHT_MOTOR_CTL   DIRECTIONS          ENA     ENB
#                                                             
#    IN1    IN2  |  IN3    IN4                             [] --- []    <-- \
#     0      1   |   0      1          CLOCKWISE              / \            (refrence point of view)
#     1      0   |   1      0          ANTICLOCKWISE      [] ----- []   <-- /

#------------------------------------------------------------------------------------------------------------------------------------------

import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)

# hardware GPIO's pin setup
ENA = 37
IN1 = 32
IN2 = 33
ENB = 31
IN3 = 38
IN4 = 40

duty_x_multiplier = 0.40
duty_z_multiplier = 0.60

GPIO.setup(ENA, GPIO.OUT)
GPIO.output(ENA,GPIO.LOW)
GPIO.setup(IN1, GPIO.OUT)
GPIO.output(IN1,GPIO.LOW)
GPIO.setup(IN2, GPIO.OUT)
GPIO.output(IN2,GPIO.LOW)

GPIO.setup(ENB, GPIO.OUT)
GPIO.output(ENB,GPIO.LOW)
GPIO.setup(IN3, GPIO.OUT)
GPIO.output(IN3,GPIO.LOW)
GPIO.setup(IN4, GPIO.OUT)
GPIO.output(IN4,GPIO.LOW)

pwm_ENA = GPIO.PWM(ENA,1000) 
pwm_ENB = GPIO.PWM(ENB,1000) 

def forward(duty):
    print("forward")
    pwm_ENA.start(duty)
    pwm_ENB.start(duty)
    GPIO.output(IN1,GPIO.LOW) # clockwise
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH) # clockwise
    GPIO.output(IN4,GPIO.LOW)

def backward(duty):
    print("backward")
    pwm_ENA.start(duty)
    pwm_ENB.start(duty)
    GPIO.output(IN1,GPIO.HIGH) # anticlockwise
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW) # anticlockwise
    GPIO.output(IN4,GPIO.HIGH)

def right(duty):
    print("right")
    pwm_ENA.start(duty)
    pwm_ENB.start(duty)
    GPIO.output(IN1,GPIO.HIGH) # anticlockwise
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.HIGH) # clockwise
    GPIO.output(IN4,GPIO.LOW)

def turn_left(duty_x,duty_z):
    global duty_x_multiplier ,duty_z_multiplier
    print("turn_left")
    duty_x = duty_x * duty_x_multiplier
    duty_z = duty_z * duty_z_multiplier
    duty = duty_x + duty_z
    pwm_ENA.start(duty)
    pwm_ENB.start(duty_x)
    GPIO.output(IN1,GPIO.LOW) # clockwise
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH) # clockwise
    GPIO.output(IN4,GPIO.LOW)

def turn_backward_left(duty_x,duty_z):
    global duty_z_multiplier,duty_x_multiplier
    print("turn_backward_left")
    duty_x = duty_x * duty_x_multiplier * 0.85
    duty_z = duty_z * duty_z_multiplier  * 1.1
    duty = duty_x + duty_z 
    pwm_ENA.start(duty)
    pwm_ENB.start(duty_x)
    GPIO.output(IN1,GPIO.HIGH) # anticlockwise
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW) # anticlockwise
    GPIO.output(IN4,GPIO.HIGH)

def left(duty):
    print("left")
    pwm_ENA.start(duty)
    pwm_ENB.start(duty)
    GPIO.output(IN1,GPIO.LOW) # clockwise
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.LOW) # anticlockwise
    GPIO.output(IN4,GPIO.HIGH)

def turn_right(duty_x,duty_z):
    global duty_x_multiplier,duty_z_multiplier
    print("turn_right")
    duty_x = duty_x * duty_x_multiplier  *  0.85
    duty_z = duty_z * duty_z_multiplier  *  1.1
    duty = duty_x + duty_z
    pwm_ENA.start(duty_x)
    pwm_ENB.start(duty)
    GPIO.output(IN1,GPIO.LOW) # clockwise
    GPIO.output(IN2,GPIO.HIGH)
    GPIO.output(IN3,GPIO.HIGH) # clockwise
    GPIO.output(IN4,GPIO.LOW)

def turn_bacward_right(duty_x,duty_z):
    global duty_z_multiplier,duty_x_multiplier
    print("turn_backward_right")
    duty_x = duty_x * duty_x_multiplier * 0.85
    duty_z = duty_z * duty_z_multiplier * 1.1
    duty = duty_x + duty_z
    pwm_ENA.start(duty_x)
    pwm_ENB.start(duty)
    GPIO.output(IN1,GPIO.HIGH) # anticlockwise
    GPIO.output(IN2,GPIO.LOW)
    GPIO.output(IN3,GPIO.LOW) # anticlockwise
    GPIO.output(IN4,GPIO.HIGH)


def stop():
    print("stop")
    pwm_ENA.stop()
    pwm_ENB.stop()
    GPIO.output(IN1,GPIO.LOW) 
    GPIO.output(IN2,GPIO.LOW) 
    GPIO.output(IN3,GPIO.LOW) 
    GPIO.output(IN4,GPIO.LOW)

