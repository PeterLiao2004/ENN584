import argparse
import time
import cv2
from pibot_client import PiBot

bot = PiBot(ip="172.19.232.188")

# Set Velocity and duration of robot movement
bot.setVelocity(50,50,5)

enc_end_left, enc_end_right = bot.getEncoders()
print(f"get encoders state at end: {enc_end_left}, {enc_end_right}")