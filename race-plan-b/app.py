import avisengine
import time
import cv2
import config
import numpy as np
from utils import make_poly
from utils import draw_lines
from utils import calc_avg_line
from utils import translate
from utils import make_hood_poly


def main():
    #Calling the class
    car = avisengine.Car()

    #connecting to the server (Simulator)
    car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)

    counter = 0

    time.sleep(3)
    steering = 0
    try:
        while(True):
            
            counter += 1 

            speed = 50

            
   

            #Get the data. Need to call it every time getting image and sensor data
            car.getData()

            #Start getting image and sensor data after 4 loops. for unclear some reason it's really important 
            if(counter > 4): 

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

                hood_poly = make_hood_poly(width,height)



                mask_shape = cv2.fillPoly(blank.copy(), pts=[poly], color=255)
        
                hood_mask_shape = cv2.fillPoly(blank.copy(),pts=[hood_poly],color=255)

                joint_hood_shape = cv2.bitwise_and(hood_mask_shape,mask_shape)

                final_mask_shape = cv2.bitwise_xor(mask_shape, joint_hood_shape)

                mask_img = cv2.bitwise_and(edge,final_mask_shape)

                gray_mask_img = cv2.bitwise_and(gray,final_mask_shape)


                lines = cv2.HoughLinesP(mask_img, rho=1, theta=np.pi/180, threshold=90, lines=np.array([]), minLineLength=10, maxLineGap=20)


                canContinue = True

                if lines is None:
                    print('no line detected!')
                    canContinue = False


                lines_img = image.copy()

                error = 0

                if canContinue == True :

                    if (len(lines) == 1):
                        lines = [lines]

                    avg_lines , error , right_err , left_err = calc_avg_line(blank.copy(),lines) 

                    if ((right_err > 1.6) & (abs(left_err) > 0.7 )) | ((abs(left_err) > 1.6) & (right_err > 0.7)):
                        error = 0

                    if ((right_err < 0.1) & (left_err != 0)) | ((abs(left_err) < 0.1) & (right_err != 0)):
                        error = 1 / error 
                        print(error)
                        speed /= 2
                    
    
                    error_trnaslated = translate(-1.5,1.5,-45,45,float(error))


                    steering = -error_trnaslated

                    lines_img = draw_lines(image.copy(),lines,(0,255,0))
                    
                #set final car steering
                car.setSteering(steering)
                
                steering = 0
                
                #set final car speed
                car.setSpeed(speed)

                #showing the opencv type image
                cv2.imshow('raw image', gray_mask_img)
                cv2.imshow('lines image', lines_img)
                # cv2.imshow('avg lines image', avg_lines_img)

                #break the loop when q pressed
                if cv2.waitKey(10) == ord('q'):
                    break


            time.sleep(0.001)
    
            
    finally:
        car.stop()
main()





