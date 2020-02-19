import numpy as np
import cv2
import glob
import time
import socket
import constants

from goprolib import GoPro

def calibration():

    print("Define Calibration Parameter...")
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    
    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((9*6,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)
    
    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    
    images = glob.glob('calib/*.png')
    i = 0
    for fname in images:
        print(len(images)-i, " images remaining")
        i+=1
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6),None)
    
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
    
            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)
    
            # Draw and display the corners
            #img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
            #cv2.imshow('img',cv2.resize(img,(int(img.shape[1]*0.4),int(img.shape[0]*0.4))))
            #cv2.waitKey(0)
    
    cv2.destroyAllWindows()
    
    if len(objpoints) > 0 :
        print("Get Optimal Parameters...")
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)
        h,  w = img.shape[:2]
        newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))
        print(roi)
        print("Done !")

        np.savez('calib/param.npz',Mi=mtx, newMi=newcameramtx, dist=dist, roi=roi, tvecs=tvecs, rvecs=rvecs)
    else:
        print("No image good enough for the calibration")

def save_image():
    gpCam = GoPro()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    t=time.time()
    gpCam.livestream("start")
    gpCam.video_settings(res='1080p', fps='30')
    gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
    cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)

    i=1;
    while(True):
        ret, frame = cap.read()
       
        cv2.imshow('frame',frame)
        key = cv2.waitKey(1) & 0xFF
        if  key == 27:
            break
        if key == ord('s'):
            nom='calib/mire_'+str(i)+'.png'
            cv2.imwrite(nom,frame)
            print("Saved image ",nom)
            i=i+1;
        if time.time() - t >= 2.5:
            sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
            t=time.time()
           
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    value = input("Do you want to define your calibration's images ?(y:1/n:0):\n")
    if value == 1:
        save_image()
        calibration()
    elif value == 0:
        calibration()
    else:
        print("Wrong value --> exit program")
        exit()