#! /usr/bin/env python
import os
import re
import cv2

DEFAULT_CAMERA_NAME = '/dev/v4l/by-id/usb-Intel_R__RealSense_TM__Depth_Camera_435i_Intel_R__RealSense_TM__Depth_Camera_435i_944123050160-video-index4'

device_num = 0
if os.path.exists(DEFAULT_CAMERA_NAME):
    device_path = os.path.realpath(DEFAULT_CAMERA_NAME)
    device_re = re.compile("\/dev\/video(\d+)")
    info = device_re.match(device_path)
    if info:
        device_num = int(info.group(1))
        print("Using default video capture device on /dev/video" + str(device_num))
cap = cv2.VideoCapture(device_num)
ret, frame = cap.read()
cv2.imshow('Display', frame)
while True:
    cv2.waitKey(1)