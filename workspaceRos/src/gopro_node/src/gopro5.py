#!/usr/bin/env python

import cv2
import time
import socket
import constants

from goprolib import GoPro

WRITE = False
gpCam = GoPro()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t=time.time()
gpCam.livestream("start")
gpCam.video_settings(res='1080p', fps='30')
gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
counter = 0

while True:
    nmat, frame = cap.read()
    cv2.imshow("GoPro OpenCV", frame)
    if WRITE == True:
        cv2.imwrite(str(counter)+".jpg", frame)
        counter += 1
        if counter >= 10:
            break
    if cv2.waitKey(1) & 0xFF == 27:
        break
    if time.time() - t >= 2.5:
        sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
        t=time.time()
# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
