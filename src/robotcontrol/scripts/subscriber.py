#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

rospy.init_node('controlnode', anonymous=False)
leftmotor_dir_pin = 2
rightmotor_dir_pin = 17
leftmotor_pwm_pin = 4
rightmotor_pwm_pin = 14

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

GPIO.setup(leftmotor_dir_pin,GPIO.OUT)
GPIO.setup(leftmotor_pwm_pin,GPIO.OUT)
GPIO.setup(rightmotor_dir_pin,GPIO.OUT)
GPIO.setup(rightmotor_pwm_pin,GPIO.OUT)

leftmotorspeed = GPIO.PWM(leftmotor_pwm_pin,200)
rightmotorspeed = GPIO.PWM(rightmotor_pwm_pin,200)
leftmotorspeed.start(0)
rightmotorspeed.start(0)

def move(linearx,angularz):
    leftmotor_speed = linearx + angularz
    rightmotor_speed = linearx - angularz
    if leftmotor_speed >= 1.0:
        leftmotor_speed = 1.0

    if rightmotor_speed >= 1.0:
        rightmotor_speed = 1.0

    if leftmotor_speed < -1.0:
        leftmotor_speed = -1.0

    if rightmotor_speed < -1.0:
        rightmotor_speed = -1.0

    leftmotor(leftmotor_speed)
    rightmotor(rightmotor_speed)


def leftmotor(speed):
    if speed <=  0:
        #print("reverse left",abs(speed))
        #rospy.loginfo("reverse left: [%f]"%(abs(speed)))
        GPIO.output(leftmotor_dir_pin,GPIO.HIGH)
        leftmotorspeed.ChangeDutyCycle(abs(speed*90))
        #time.sleep(10)
    if speed > 0 :
        #print+("forward left",abs(speed))
        #rospy.loginfo("forward left: [%f]"%(abs(speed)))
        GPIO.output(leftmotor_dir_pin,GPIO.LOW)
        leftmotorspeed.ChangeDutyCycle(abs(speed*90))
        #time.sleep(10)

    


def rightmotor(speed):
    if speed <= 0 :
        #print("reverse right",abs(speed))
        #rospy.loginfo("reverse right: [%f]"%(abs(speed)))
        GPIO.output(rightmotor_dir_pin,GPIO.HIGH)
        rightmotorspeed.ChangeDutyCycle(abs(speed*90))

    if speed > 0:
        #print("forward right",abs(speed))
        #rospy.loginfo("forward right: [%f]"%(abs(speed)))
        GPIO.output(rightmotor_dir_pin,GPIO.LOW)
        rightmotorspeed.ChangeDutyCycle(abs(speed*90))
        #time.sleep(10)

def callback(msg):

	
        move(msg.linear.x,msg.angular.z)

def listener_and_pub():
	rospy.Subscriber("/cmd_vel", Twist, callback) 
	rospy.spin()
	
if __name__ == '__main__':
	try:
		listener_and_pub()
	except rospy.ROSInterruptException:
		pass
