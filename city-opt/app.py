import avisengine
import time
import cv2
import config
import numpy as np

from utils import *
from test import test_sign

def main():
    #right sign == 2
    #straight sign == 4
    #left sign == 3
    #stop sign == 5
    tags_list = {
        "1":"straight",
        "2":"right",
        "3":"left",
        "4":"straight",
        "5":"stop"
    }
    
    #this object needs to configed accorded to map
    #map 2 tags_list
    tags_list = {
        "1":"straight",
        "2":"right",
        "3":"left",
        "4":"straight",
        "5":"stop"
    }
    


    actions_list = {
        "right" : [
            {
                "speed": 35,
                "steering" : 35,
                "time" : 2
            },
            {
                "speed": 20,
                "steering" : 75,
                "time" : 3.2
            }
        ],
        "straight":[
            {
                "speed":35,
                "steering":0,
                "time":4.3
            },
        ],
        "left":[
            {
                "speed": 30,
                "steering" : 10,
                "time" : 1.5
            },
            {
                "speed": 10,
                "steering" : -55,
                "time" : 5
            }
        ]

    }

    actions_delay = {
        "right":0,
        "left":0.35,
        "straight":0.2,
        "stop":0.75
    }

    test_sign_mode = False

    #Calling the class
    car = avisengine.Car()

    #connecting to the server (Simulator)
    car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)

    counter = 0

    is_auto_mode_in_use = False

    time.sleep(3)
    steering = 0

    turn_mode_repeated = 0

    diff_diff = 0

    try:
        while(True):
            
            counter += 1    

            speed = 30

            car.getData()

            if(counter > 4): 

                car_speed = car.getSpeed()
                
                _image = car.getImage()

                if _image is None:
                    print('None image received!!    ')
                    continue

                
                image = _image.copy()
            
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

                blank = np.zeros_like(gray)	


                height, width = gray.shape

#                         (400, 350),
#                         (400 , 150),
#                         (512 ,150),
#                         (512, 350)

                sign_poly = np.array([
                    [
                        (250        , height),
                        (250         , 0),
                        (512 ,0),
                        (512, height)
                    ]
                    ])

                sign_mask_shape = cv2.fillPoly(blank.copy(), pts=[sign_poly], color=255)

                sign_mask_img = cv2.bitwise_and(gray,sign_mask_shape)

                sign_mask_img_overlay  = cv2.bitwise_or(gray,sign_mask_shape)
                
                if test_sign_mode:
                    
                    test_sign(sign_mask_img)

                    car.setSpeed(0)

                    continue


                arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
                arucoParams = cv2.aruco.DetectorParameters()
                (corners, ids, rejected) = cv2.aruco.detectMarkers(sign_mask_img, arucoDict,
                    parameters=arucoParams)
                
                auto_mode = False

                if ids is not None:

                    # print(ids[0][0])

                    # break

                    action = tags_list[str(int(ids[0][0]))]

                    smalest_x = 100000000
                    biggest_x = -100000000

                    i = 50

                    for (x,y) in corners[0][0]:
                        if x < smalest_x:
                            smalest_x = x
                        if x > biggest_x:
                            biggest_x = x
                    print("rejected ",action)

                    diff_x = biggest_x - smalest_x

                    if action == "stop":
                        i = 35

                    if action == "left":
                        i = 52

                    if action == "straight":
                        i = 55

                    if (diff_x > 15) & (diff_x < i):
                        speed = 15

                    if (diff_x > i) :
                        
                        diff_diff = diff_x - i if action != "stop" else 0

                        if action == "left":
                            diff_diff = diff_diff if diff_diff < 5 else diff_diff -3 
                            diff_diff = diff_diff if diff_diff < 9 else diff_diff -5 

                        print(diff_x)



                        auto_mode = True

                if auto_mode :

                    car.setSteering(0)

                    action = tags_list[str(int(ids[0][0]))]


                    delay = actions_delay[action]
                   
                    is_force_brake_mode = False

                    time.sleep(delay if delay  > 0 else 0)


                    handle_brake(car,car_speed + car_speed * diff_diff / 12 if diff_diff > 4 else car_speed)
        
                    time.sleep(3 if diff_diff < 1 else 3 + 1.5)

                    for instruction in actions_list[action]:
                        car.setSpeed(instruction["speed"])
                        car.setSteering(instruction["steering"])
                        time.sleep(instruction["time"])

                    diff_diff = 0

                    car.getData()

                    continue

                else :    

                    tran_in , tran_out = -45, 45

                    blur = cv2.GaussianBlur(gray.copy(), (3,15), 0)

                    edge = cv2.Canny(blur, 135, 150)

                    poly = make_poly(width,height)

                    hood_poly = make_hood_poly(width,height)  

                    mask_shape = cv2.fillPoly(blank.copy(), pts=[poly], color=255)
            
                    hood_mask_shape = cv2.fillPoly(blank.copy(),pts=[hood_poly],color=255)

                    joint_hood_shape = cv2.bitwise_and(hood_mask_shape,mask_shape)

                    final_mask_shape = cv2.bitwise_xor(mask_shape, joint_hood_shape)

                    mask_img = cv2.bitwise_and(edge,final_mask_shape)

                    gray_mask_img = cv2.bitwise_and(gray,final_mask_shape)

                    lines = cv2.HoughLinesP(mask_img, rho=1, theta=np.pi/180 , threshold=90, lines=np.array([]), minLineLength=25, maxLineGap=20)

                    canContinue = True
                    
                    if lines is None:
                        print('no line detected!')
                        canContinue = False

                    lines_img = image.copy()


                    error = 0


                    if canContinue == True :

                        if (len(lines) == 1):
                            lines = [lines]

                        avg_lines , error , right_error , left_error  = calc_avg_line(blank.copy(),lines) 


                        if (right_error > 1.3 )& (abs(left_error) > 1.3):
                            error = 0

                        if (right_error == 0) :
                            turn_mode_repeated += 1
                            speed = 5
                            error = - 4
                            if turn_mode_repeated > 12:
                                speed = 12
                        else:
                            turn_mode_repeated = 0



    
                        if (left_error == 0) & (right_error > 1):
                            turn_mode_repeated += 1 
                            speed = 3
                            error = 4
                            if turn_mode_repeated > 5:
                                error = 5

                        if (abs(error) < 0.5) & (error != 0):
                            error = 1 / error

                        error_trnaslated = translate(-1.5,1.5,-45,45,float(error))

                        steering = -error_trnaslated


                        lines_img = draw_lines(image.copy(),lines,(0,255,0))
                    
                    car.setSpeed(speed)
                    
                    car.setSteering(steering)                

                cv2.imshow('image', lines_img)
                
                if cv2.waitKey(10) == ord('q'):
                    break
            time.sleep(0.001)
    
            
    finally:
        car.stop()
main()









