import avisengine
import config
import time
import cv2
import numpy as np
from required import *

# Creating an instance of the Car class
car = avisengine.Car()

# Connecting to the server (Simulator)
car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)

# Counter variable
counter = 0
steering = 0

debug_mode = False

# Sleep for 3 seconds to make sure that client connected to the simulator 
time.sleep(3)

try:
    while(True):

        # Counting the loops
        counter = counter + 1

        # Set the power of the engine the car to 20, Negative number for reverse move, Range [-100,100]
        car.setSpeed(50)

        # Set the Steering of the car -10 degree from center, results the car to steer to the left
        car.setSteering(steering)
        
        # Set the angle between sensor rays to 45 degrees, Use this only if you want to set it from python client
        # Notice: Once it is set from the client, it cannot be changed using the GUI
        car.setSensorAngle(45) 

        # Get the data. Need to call it every time getting image and sensor data
        car.getData()

        # Start getting image and sensor data after 4 loops
        if(counter > 4):

            # Returns a list with three items which the 1st one is Left sensor data\
            # the 2nd one is the Middle Sensor data, and the 3rd is the Right one.
            sensors = car.getSensors() 

            # Returns an opencv image type array. if you use PIL you need to invert the color channels.
            _image = car.getImage()
            
            if _image is None:
                print('image not received!')
                continue

            image = _image.copy()

            # Returns an integer which is the real time car speed in KMH
            carSpeed = car.getSpeed()

            # get a gray image from the original image
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # make our grayimage to blur
            blur = cv2.GaussianBlur(gray, (3,15), 0)

            # get our image's edges
            edge = cv2.Canny(blur, 200, 300)

            blank = np.zeros_like(edge)	

            width, height = edge.shape	

            poly = make_poly(width,height)

            mask_shape = cv2.fillPoly(blank, pts=[poly], color=255)
     
            mask_img = cv2.bitwise_and(edge,mask_shape)

            lines = cv2.HoughLinesP(mask_img, rho=3, theta=np.pi/90, threshold=20, lines=np.array([]), minLineLength=10, maxLineGap=5)

            color = (0,255,0)

            # Show the results if we had lines
            if lines is None:
                print('no line detected!')
                continue

            else:
                car.setSpeed(5)

                if (len(lines) == 1):
                    lines = [lines]

                avg_lines , error = calc_avg_line(blank.copy(),lines) 
        

            steering = translate(-1.5,1.5,-4.5,4.5,float(error))

            lines_img = draw_lines(image,lines,(0,255,0))


            if(debug_mode):
                print(f"Speed : {carSpeed}") 
                print(f'Left : {str(sensors[0])} | Middle : {str(sensors[1])} | Right : {str(sensors[2])}')
            
            # Showing the opencv type image
            cv2.imshow('frames', image)
            cv2.imshow('canny', edge)


            if cv2.waitKey(10) == ord('q'):
                break

            time.sleep(0.001)

finally:
    car.stop()