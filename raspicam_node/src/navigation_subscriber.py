#!/usr/bin/env python3

# publisher command :-
# rosrun teleop_twist_keyboard teleop_twist_keyboard.py _speed:=100 _turn:=100
# sudo chown root.gpio /dev/gpiomem
# sudo chmod g+rw /dev/gpiomem


import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
import time



def forward(left, right):
    print("forward, Left- ",left,"Right- ",right)
    L_MOTOR1.ChangeDutyCycle(left)
    R_MOTOR1.ChangeDutyCycle(right)
def backward(left, right):
    print("backward left- ",left,"Right- ",right)
    R_MOTOR2.ChangeDutyCycle(left)
    L_MOTOR2.ChangeDutyCycle(right)
    
def stop():
    L_MOTOR1.ChangeDutyCycle(0)
    R_MOTOR1.ChangeDutyCycle(0)
    L_MOTOR2.ChangeDutyCycle(0)
    R_MOTOR2.ChangeDutyCycle(0)

def callback(msg):
    #forward(100,100)
    #time.sleep(2)
    print("callback time")
    print(f" linear_X : {msg.linear.x} \n angular_z : {msg.angular.z}")
    take_action(msg.linear.x,msg.angular.z)

def take_action(X,Z):
    print("take_action called")
    if X > 0:
    	print("x>0")
    	left = X - (Z/2)
    	right = X + (Z/2)
    	forward(left,right)
    elif X < 0:
    	print("x<0")
    	left = -X -Z
    	right = -X  
    	backward(left,right)
    elif X == 0 and Z != 0:
    	X = 45
    	left = X - Z
    	right = X + Z 
    	forward(left,right)
    elif X == 0:
    	stop()
    	print("x==0")

def main():
    print("running")
    rospy.init_node('navigation_subscriber')
    print("init done")
    rospy.Subscriber("/cmd_vel",Twist,callback)
    print("started subscribing")
    rospy.spin()


if __name__ == '__main__':
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
    L_MOTOR1 = GPIO.PWM(L_PWM_PIN1, 100) 
    R_MOTOR1 = GPIO.PWM(R_PWM_PIN1, 100)
    L_MOTOR2 = GPIO.PWM(L_PWM_PIN2, 100)
    R_MOTOR2 = GPIO.PWM(R_PWM_PIN2, 100) 
    
    L_MOTOR1.start(0)
    R_MOTOR1.start(0)
    L_MOTOR2.start(0)
    R_MOTOR2.start(0)
    main()

