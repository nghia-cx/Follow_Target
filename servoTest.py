#!/usr/bin/env python
#
#  Pan Tilt Servo Control 
#  Execute with parameter ==> sudo python3 servoCtrl.py <pan_angle> <tilt_angle>
#
#  MJRoBot.org 01Feb18
import curses
import cv2
from time import sleep
import RPi.GPIO as GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
import time


# get the curses screen window
'''screen = curses.initscr()

# turn off input echoing
curses.noecho()

# respond to keys immediately (don't wait for enter)
curses.cbreak()

# map arrow keys to special values
screen.keypad(True)'''



def setServoAngle(servo, angle):
	assert angle >=0 and angle <= 180
	pwm = GPIO.PWM(servo, 50)
	pwm.start(8)
	dutyCycle = angle / 18. + 3.
	pwm.ChangeDutyCycle(dutyCycle)
	sleep(0.3)
	pwm.stop()
	

if __name__ == '__main__':  
    '''for i in range (30, 160, 10):
        setServoAngle(pan, i)
        setServoAngle(tilt, i)
    
    for i in range (150, 30, -10):
        setServoAngle(pan, i)
        setServoAngle(tilt, i)
        
    setServoAngle(pan, 90)
    setServoAngle(tilt, 90)    
    GPIO.cleanup()'''
    
    pan = 27
    tilt = 17
    
    GPIO.setup(tilt, GPIO.OUT) # white => TILT
    GPIO.setup(pan, GPIO.OUT) # gray ==> PAN
    
    max_PAN      = 180
    max_TILT     = 145
    min_PAN      = 0
    min_TILT     = 0
        
    step_PAN     = 10
    step_TILT    = 10
    current_PAN  = 90
    current_TILT = 90
    
    setServoAngle(pan, 110)
    setServoAngle(tilt, 100) 
   
    try:
        cap = cv2.VideoCapture(0)
        while True:
            ret, frame = cap.read()
            frame = cv2.resize(frame, (640,480))
            if not ret:
                continue
            cv2.imshow("Frame",frame)
            #char = screen.getch()
            char = cv2.waitKey(1) & 0xFF
            if char == ord('q'):
                #if q is pressed quit
                break
                
            #elif char == curses.KEY_RIGHT:
            elif char == 81:
                #screen.addstr(0, 0, 'right ')
                current_PAN = max(min_PAN, current_PAN - step_PAN)
                setServoAngle(pan, current_PAN)
                time.sleep(0.001)
                
            #elif char == curses.KEY_LEFT:
            elif char == 83:
                #screen.addstr(0, 0, 'left ')
                current_PAN = min(max_PAN, current_PAN + step_PAN)
                setServoAngle(pan, current_PAN) 
                time.sleep(0.001)
                
            #elif char == curses.KEY_UP:
            elif char == 84:
                #screen.addstr(0, 0, 'up ')
                current_TILT = max(min_TILT, current_TILT - step_TILT)
                setServoAngle(tilt, current_TILT) 
                time.sleep(0.001)
                
            #elif char == curses.KEY_DOWN:
            elif char == 82:
                #screen.addstr(0, 0, 'up ')
                current_TILT = min(max_TILT, current_TILT + step_TILT)
                setServoAngle(tilt, current_TILT) 
                time.sleep(0.001)
    finally:
        # shut down cleanly
        GPIO.cleanup()
        #curses.nocbreak(); screen.keypad(0); curses.echo()
        #curses.endwin()
