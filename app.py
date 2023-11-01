import FiraAuto
import time
import cv2
import numpy as np
from utils import make_poly
from utils import draw_lines
from utils import calc_avg_line
#Calling the class
car = FiraAuto.car()

#connecting to the server (Simulator)
car.connect("127.0.0.1", 25001)

#Counter variable
counter = 0

debug_mode = False
#sleep for 2 second to make sure that client connected to the simulator 
time.sleep(2)
global steering
steering = 0
try:
    while(True):

        print(steering)

        #Counting the loops
        
        counter = counter + 1

        #Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
        car.setSpeed(60)

        #Set the Steering of the car -10 degree from center
        car.setSteering(steering)

        #Get the data. Need to call it every time getting image and sensor data
        car.getData()

        #Start getting image and sensor data after 4 loops. for unclear some reason it's really important 
        if(counter > 4):
            #returns a list with three items which the 1st one is Left sensor data, the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
            sensors = car.getSensors() 
            #EX) sensors[0] returns an int for left sensor data in cm

            #returns an opencv image type array. if you use PIL you need to invert the color channels.
            _image = car.getImage()

            if _image is None:
                print('None image received!!    ')
                continue

            carSpeed = car.getSpeed()
            
            image = _image.copy()

            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            blur = cv2.GaussianBlur(gray, (3,15), 0)

            edge = cv2.Canny(blur, 200, 300)

            blank = np.zeros_like(edge)	

            height, width = edge.shape	

            poly = make_poly(width,height)

            mask_shape = cv2.fillPoly(blank, pts=[poly], color=255)
     
            mask_img = cv2.bitwise_and(edge,mask_shape)

            lines = cv2.HoughLinesP(mask_img, rho=3, theta=np.pi/90, threshold=20, lines=np.array([]), minLineLength=10, maxLineGap=5)


            if lines is None:
                print('no line detected!')
                continue

            if (len(lines) == 1):
                lines = [lines]

            avg_lines , error = calc_avg_line(blank.copy(),lines) 

            steering = -error

            lines_img = draw_lines(image.copy(),lines,(0,255,0))

            #Don't print data for better performance
            if(debug_mode):
                print("Speed : ",carSpeed) 
                #currently the angle between the sensors is 30 degree TODO : be able to change that from conf.py
                print("Left : " + str(sensors[0]) + "   |   " + "Left : " + str(sensors[1])  +"   |   " + "Left : " + str(sensors[2]))

            #showing the opencv type image
            cv2.imshow('frames', lines_img)
            #break the loop when q pressed
            if cv2.waitKey(10) == ord('q'):
                break
            time.sleep(0.001)
        #A brief sleep to make sure everything 
        
finally:
    car.stop()





