import numpy as np
import cv2
import math
import RPi.GPIO as GPIO

cap = cv2.VideoCapture(0)
servoPIN = 17
servoPIN2 = 18
servoPIN3 = 27

GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)
GPIO.setup(servoPIN2, GPIO.OUT)
GPIO.setup(servoPIN3, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50) # GPIO 17 for PWM with 50Hz
p.start(5) # Initialization
p2 = GPIO.PWM(servoPIN2, 50) # GPIO 18 for PWM with 50Hz
p2.start(5) # Initialization

img = 0
font = cv2.FONT_HERSHEY_SIMPLEX
final_points = [(0,0),(0,0),(0,0)]
print(cap.get(3), cap.get(4))
lower_y = np.array([240, 240, 240])
upper_y = np.array([255, 255, 255])

def translate(value, leftMin, leftMax, rightMin, rightMax):
        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin

        valueScaled = float(value - leftMin)/ float(leftSpan)

        return rightMin + (valueScaled * rightSpan)

def computeAngel(cod1, cod2, cod3, frame):
        x1, y1 = cod1
        x2, y2 = cod2
        x3, y3 = cod3

        m1 = 0
        m2 = 0
        b = x2 - x1
        if (b != 0):
                m2 = ((y2 - y1)/b)
                o1 = math.atan((m2 - m1)/(1+m2*m1))
                o1 = 90 - (o1 * (180/3.141))                                                                    
        else:
                o1 = 90
        #print("Theta 1 : ",o1)


        c = x3 - x2
        if (c != 0):
                m3 = ((y3 - y2)/c)
                o2 = math.atan((m3 - m2)/(1+m3*m2))
                o2 = (o2 * (180/3.141))
                if (o2 < 0):
                        o2 = (o2 + 180)
        else:
                o2 = 90
        #print("Theta 2 : ",o2)
        aTheta = 7
        bTheta = 2.5
        if((int(o1) > 10) and (int(o1) < 60)):
                aTheta = translate(int(o1), 10, 60, 12, 7)
                print(aTheta)
                p.ChangeDutyCycle(aTheta)

        if((int(o2) > 60) and (int(o2) < 180)):
                bTheta = translate(int(o2), 60, 180, 6, 2.5)
                print(bTheta)
                p2.ChangeDutyCycle(bTheta)
                
        cv2.putText(frame,'Theta 1: {}'.format(int(o1)),(10,80), font, 1,(255,255,255),2,cv2.LINE_AA)
        cv2.putText(frame,'Theta 2: {}'.format(int(o2)),(10,110), font, 1,(255,255,255),2,cv2.LINE_AA)
                
kernel = np.ones((2,2),np.uint8)
while(cap.isOpened()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        #re, frame1 = cap1.read()
        #frame1 = cv2.cvtColor(frame1, cv2.COLOR_BGR2HSV)
        # Our operations on the frame come here
        # Display the resulting frame
        thresh = cv2.inRange(frame,lower_y,upper_y)
        thresh - cv2.dilate(thresh, kernel, 2)
        thresh = cv2.erode(thresh, kernel, 2)
        #canny = cv2.Canny(thresh, 50, 150, 5)
        _, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        #print(len(contours))
        if len(contours) != 0:
                for j,i in enumerate(contours):
                        (x,y,w,h) = cv2.boundingRect(i)
                        #asp = w/h
                        #if((asp > 0.1 and asp <0.7) or (asp >1.1  and asp<1.7)):
                        #if(w>10 and w<30 and h>30 and h<50):
                        xc = int(x+(w/2))
                        yc = int(y+(h/2))
                        #frame = cv2.rectangle(frame,(x,y),(x+w,y+h),(255,0,0),3)
                        #frame = cv2.circle(frame, (xc,yc), 10, (255,0,0), -1)
                        if (j>-1 and j<3):
                                if (xc != 0 and yc != 0):
                                        if (xc < 320 and yc <240):
                                                if (yc > 100):
                                                        final_points[1] = (xc, yc)
                                                else:
                                                        final_points[0] = (xc, yc)
                                        else:
                                                final_points[2] = (xc, yc)
                                        computeAngel(final_points[0], final_points[1], final_points[2], frame)
                                frame = cv2.circle(frame, final_points[0], 10, (0,255,0), -1)
                                frame = cv2.circle(frame, final_points[1], 10, (0,255,0), -1)
                                frame = cv2.circle(frame, final_points[2], 10, (0,255,0), -1)
                                frame = cv2.line(frame, final_points[0], final_points[1],
                                (0,255,0),2)
                                frame = cv2.line(frame, final_points[1], final_points[2],
                                (0,255,0),2)
        else: 
                cv2.putText(frame,'Nothing is detected',(10,100), font, 1,(0,0,255),4,cv2.LINE_AA)
                frame = cv2.circle(frame, final_points[0], 10, (0,255,0), -1)
                frame = cv2.circle(frame, final_points[1], 10, (0,255,0), -1)
                frame = cv2.circle(frame, final_points[2], 10, (0,255,0), -1)
                frame = cv2.line(frame, final_points[0], final_points[1],
                (0,255,0),2)
                frame = cv2.line(frame, final_points[1], final_points[2],
                (0,255,0),2)
                #computeAngel(final_points[0], final_points[1], final_points[2])        


        cv2.line(frame, (320, 480), (320, 0), (0,255,0), 2)
        cv2.line(frame, (640, 240), (0, 240), (0,255,0), 2)
        cv2.imshow('External Cam',frame)

        #print(final_points)
        #cv2.imshow('Integrated Cam',frame1)
        if cv2.waitKey(1) & 0xFF == ord('q'):
                break
# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
p.stop()
GPIO.cleanup()

