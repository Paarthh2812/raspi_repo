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
import time

# hardware GPIO's pin setup
EN_A = 31
EN_B = 37
L_PWM_PIN1 = 38
L_PWM_PIN2 = 40
R_PWM_PIN2 = 32
R_PWM_PIN1 = 33

GPIO.setmode(GPIO.BOARD)
GPIO.setup(EN_A, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(EN_B, GPIO.OUT, initial=GPIO.HIGH)
GPIO.setup(R_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(R_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(L_PWM_PIN1, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(L_PWM_PIN2, GPIO.OUT, initial=GPIO.LOW)

# setting initial PWM frequency for all 4 pins
L_MOTOR1 = GPIO.PWM(L_PWM_PIN1, 100) 
R_MOTOR1 = GPIO.PWM(R_PWM_PIN1, 100)
L_MOTOR2 = GPIO.PWM(L_PWM_PIN2, 100)
R_MOTOR2 = GPIO.PWM(R_PWM_PIN2, 100) 
    
# setting initial speed (duty cycle) for each pin as 0
L_MOTOR1.start(0)
R_MOTOR1.start(0)
L_MOTOR2.start(0)
R_MOTOR2.start(0)


def forward(left, right):
    print("forward, Left- ",left,"Right- ",right)
    L_MOTOR1.ChangeDutyCycle(left)
    R_MOTOR1.ChangeDutyCycle(right)
def backward(left, right):
    print("backward left- ",left,"Right- ",right)
    R_MOTOR2.ChangeDutyCycle(left)
    L_MOTOR2.ChangeDutyCycle(right)
    
def stop():
    L_MOTOR1.stop()
    R_MOTOR1.stop()
    L_MOTOR2.stop()
    R_MOTOR2.stop()
    GPIO.cleanup()
