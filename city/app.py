import avisengine
import time
import cv2
import config
import numpy as np
from utils import handle_brake, make_poly
from utils import draw_lines
from utils import calc_avg_line
from utils import translate
from utils import make_hood_poly
from utils import handle_brake_intensity


def main():
    #right sign == 2
    #straight sign == 4
    #left sign == 3
    #stop sign == 5
    tags_list = {"2":"right",
                 "3":"left",
                 "4":"straight",
                 "5":"stop"
    }
    
    actions_list = {
        "right" : [
            {
                "speed": 30,
                "steering" : 0,
                "time" : 2
            },
            {
                "speed": 5,
                "steering" : 100,
                "time" : 5.8
            }
        ],
        "straight":[
            {
                "speed":35,
                "steering":0,
                "time":5.5
            },
        ],
        "left":[
            {
                "speed": 30,
                "steering" : 0,
                "time" : 2
            },
            {
                "speed": 10,
                "steering" : -55,
                "time" : 9
            }
        ]

    }

    #Calling the class
    car = avisengine.Car()

    #connecting to the server (Simulator)
    car.connect(config.SIMULATOR_IP, config.SIMULATOR_PORT)

    counter = 0

    is_auto_mode_in_use = False

    time.sleep(3)
    steering = 0
    try:
        while(True):
            
            counter += 1 

            speed = 20

            
            #Get the data. Need to call it every time getting image and sensor data
            car.getData()

            #Start getting image and sensor data after 4 loops. for unclear some reason it's really important 
            if(counter > 4): 

                car_speed = car.getSpeed()
                
                #returns an opencv image type array. if you use PIL you need to invert the color channels.
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
                        (250        , 350),
                        (250         , 150),
                        (512 ,150),
                        (512, 350)
                    ]
                    ])

                sign_mask_shape = cv2.fillPoly(blank.copy(), pts=[sign_poly], color=255)

                sign_mask_img = cv2.bitwise_and(gray,sign_mask_shape)

                sign_mask_img_overlay  = cv2.bitwise_or(gray,sign_mask_shape)

                arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
                arucoParams = cv2.aruco.DetectorParameters()
                (corners, ids, rejected) = cv2.aruco.detectMarkers(sign_mask_img, arucoDict,
                    parameters=arucoParams)
                
                auto_mode = False

                if ids is not None:
                    
                    smalest_y = 100000000
                    smalest_x = 100000000
                    biggest_x = -100000000

                    for (x,y) in corners[0][0]:
                        if y < smalest_y:
                            smalest_y = y
                        if x < smalest_x:
                            smalest_x = x
                        if x > biggest_x:
                            biggest_x = x
                    print("unval ",tags_list[str(int(ids[0][0]))])
                    if (biggest_x - smalest_x > 45) :
                        print(biggest_x,smalest_x)
                        print(tags_list[str(int(ids[0][0]))])

                        auto_mode = True



                # car.setSpeed(0)
                if auto_mode :


                    # if corners:
                    #     corners_img = cv2.rectangle(image.copy(),(int(corners[0][0][0][0]),int(corners[0][0][0][1])),(int(corners[0][0][2][0]),int(corners[0][0][2][1])),color=(255,0,0),thickness=2)
                    #     # cv2.imshow('c image', corners_img)
                    
                    # cv2.imshow("image",image)
                   
                    # cv2.imshow("sign mask", final_sign_mask_img)

                    action = tags_list[str(int(ids[0][0]))]
                    
                    if action == "right":

                        handle_brake(car,car_speed)
                        
                        for instruction in actions_list[action]:
                            car.setSpeed(instruction["speed"])
                            car.setSteering(instruction["steering"])
                            time.sleep(instruction["time"])

                    if action == "straight":
                        handle_brake(car,car_speed)

                        for instruction in actions_list[action]:
                            car.setSpeed(instruction["speed"])
                            car.setSteering(instruction["steering"])
                            time.sleep(instruction["time"])

                        car.setSpeed(0)

                    if action == "left":

                        handle_brake(car,car_speed)


                        for instruction in actions_list[action]:
                            car.setSpeed(instruction["speed"])
                            car.setSteering(instruction["steering"])
                            time.sleep(instruction["time"])


                    car.getData()

                    continue

                else :    

                    blur = cv2.GaussianBlur(gray.copy(), (3,15), 0)

                    edge = cv2.Canny(blur, 200, 300)

                    poly = make_poly(width,height)

                    hood_poly = make_hood_poly(width,height)  

                    mask_shape = cv2.fillPoly(blank.copy(), pts=[poly], color=255)
            
                    hood_mask_shape = cv2.fillPoly(blank.copy(),pts=[hood_poly],color=255)

                    joint_hood_shape = cv2.bitwise_and(hood_mask_shape,mask_shape)

                    final_mask_shape = cv2.bitwise_xor(mask_shape, joint_hood_shape)

                    mask_img = cv2.bitwise_and(edge,final_mask_shape)

                    gray_mask_img = cv2.bitwise_and(gray,final_mask_shape)


                    # lines = cv2.HoughLinesP(mask_img, rho=5, theta=np.pi/180, threshold=90, lines=np.array([]), minLineLength=10, maxLineGap=20)
                    lines = cv2.HoughLinesP(mask_img, rho=5, theta=np.pi/180 , threshold=90, lines=np.array([]), minLineLength=10, maxLineGap=20)


                    if lines is None:
                        print('no line detected!')
                        continue

                    lines_img = image.copy()


                    error = 0

                    canContinue = True

                    if canContinue == True :

                        if (len(lines) == 1):
                            lines = [lines]

                        avg_lines , error , right_error , left_error  = calc_avg_line(blank.copy(),lines) 


                        if (right_error > 1.3 )& (abs(left_error) > 1.3):
                            error = 0

                        if (right_error == 0):
                            error = -3
    
                        if (left_error == 0):
                            speed = 7
                            error = 6

                        if (abs(error) < 0.5) & (error != 0):
                            error = 1 / error

                        error_trnaslated = translate(-1.5,1.5,-45,45,float(error))

                        steering = -error_trnaslated

                        print(right_error,left_error)

                        lines_img = draw_lines(image.copy(),lines,(0,255,0))

                        
                    
                    #set final car steering
                    
                    
                    # speed = 0
                    # steering = 0
                    
                    #set final car speed
                    car.setSpeed(speed)
                    
                    car.setSteering(steering)                

                # cv2.imshow('avg lines image', avg_lines_img)
                
                cv2.imshow("mask",sign_mask_img)

                #break the loop when q pressed
                if cv2.waitKey(10) == ord('q'):
                    break

                # end_time  = time.time()

                # print('this proccess take ',end_time - start_time,'seconds')
                # print(error)

            time.sleep(0.001)
    
            
    finally:
        car.stop()
main()









