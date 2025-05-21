import cv2 as cv
import numpy as np
import serial
import time
import logging


import camera

camera = camera.Camera()
camera.connect(1, 1280, 720)
camera.initSlider()

while True:
    camera.capture(250)