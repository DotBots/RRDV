# import the necessary packages
import numpy as np
import argparse
import cv2
import sys

# load the ArUCo dictionary
arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

tag = np.zeros((300, 300, 1), dtype="uint8")
cv2.aruco.drawMarker(arucoDict, 15, 300, tag, 1)

cv2.imshow("ArUCo Tag", tag)
cv2.waitKey(0)