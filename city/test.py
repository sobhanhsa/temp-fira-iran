import avisengine
import time
import cv2
import config
import numpy as np



def test_sign(sign_image):

    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    arucoParams = cv2.aruco.DetectorParameters()
    (corners, ids, rejected) = cv2.aruco.detectMarkers(sign_image, arucoDict,
    parameters=arucoParams)

    if ids is not None:
        id = int(ids[0][0])
        print("sign id",id)
    else:
        print("no sign detected")
