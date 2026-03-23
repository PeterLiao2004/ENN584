import argparse
import time
import cv2
from pibot_client import PiBot

pb = PiBot(ip="172.19.232.188")

img = pb.getImage() # Grab an image from the robot
cv2.imshow("windowName",img) # Display it
cv2.waitKey(500)