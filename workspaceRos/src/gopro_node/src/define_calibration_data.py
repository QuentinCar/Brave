import numpy as np
import cv2
import glob
import time
import socket
import constants

from goprolib import GoPro

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


def calibration(n_boards, board_w, board_h, board_dim):
    #Initializing variables
    board_n = board_w * board_h
    opts = []
    ipts = []
    npts = np.zeros((n_boards, 1), np.int32)
    intrinsic_matrix = np.zeros((3, 3), np.float32)
    distCoeffs = np.zeros((5, 1), np.float32)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)

    # prepare object points based on the actual dimensions of the calibration board
    # like (0,0,0), (25,0,0), (50,0,0) ....,(200,125,0)
    objp = np.zeros((board_h*board_w,3), np.float32)
    objp[:,:2] = np.mgrid[0:(board_w*board_dim):board_dim,0:(board_h*board_dim):board_dim].T.reshape(-1,2)

    #Loop through the images.  Find checkerboard corners and save the data to ipts.
    for i in range(1, n_boards + 1):
    
        #Loading images
        print 'Loading... calib/mire_' + str(i) + '.png' 
        image = cv2.imread('calib/mire_' + str(i) + '.png')
    
        #Converting to grayscale
        grey_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        #Find chessboard corners
        found, corners = cv2.findChessboardCorners(grey_image, (board_w,board_h),cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        print (found)

        if found == True:

            #Add the "true" checkerboard corners
            opts.append(objp)

            #Improve the accuracy of the checkerboard corners found in the image and save them to the ipts variable.
            cv2.cornerSubPix(grey_image, corners, (20, 20), (-1, -1), criteria)
            ipts.append(corners)

            #Draw chessboard corners
            cv2.drawChessboardCorners(image, (board_w, board_h), corners, found)
        
            #Show the image with the chessboard corners overlaid.
            cv2.imshow("Corners", image)

        char = cv2.waitKey(0)

    cv2.destroyWindow("Corners") 
    
    print ''
    print 'Finished processes images.'

    #Calibrate the camera
    print 'Running Calibrations...'
    print(' ')
    ret, intrinsic_matrix, distCoeff, rvecs, tvecs = cv2.calibrateCamera(opts, ipts, grey_image.shape[::-1],None,None)

    #Save matrices
    print('Intrinsic Matrix: ')
    print(str(intrinsic_matrix))
    print(' ')
    print('Distortion Coefficients: ')
    print(str(distCoeff))
    print(' ') 


    #Calculate the total reprojection error.  The closer to zero the better.
    tot_error = 0
    for i in xrange(len(opts)):
        imgpoints2, _ = cv2.projectPoints(opts[i], rvecs[i], tvecs[i], intrinsic_matrix, distCoeff)
        error = cv2.norm(ipts[i],imgpoints2, cv2.NORM_L2)/len(imgpoints2)
        tot_error += error

    print "total reprojection error: ", tot_error/len(opts)

    #Undistort Images

    #Scale the images and create a rectification map.
    newMat, ROI = cv2.getOptimalNewCameraMatrix(intrinsic_matrix, distCoeff, image_size, alpha = crop, centerPrincipalPoint = 1)
    mapx, mapy = cv2.initUndistortRectifyMap(intrinsic_matrix, distCoeff, None, newMat, image_size, m1type = cv2.CV_32FC1)
    
    #Save data
    print 'Saving data file...'
    np.savez('calib/calibration_data', distCoeff=distCoeff, intrinsic_matrix=intrinsic_matrix, mapx=mapx, mapy=mapy)
    print 'Calibration complete'
  
    for i in range(1, n_boards + 1):
    
        #Loading images
        print 'Loading... calib/mire_' + str(i) + '.png' 
        image = cv2.imread('calib/mire_' + str(i) + '.png')

        # undistort
        dst = cv2.remap(image, mapx, mapy, cv2.INTER_LINEAR)

        cv2.imshow('Undisorted Image',dst)

        char = cv2.waitKey(0)
    

    cv2.destroyAllWindows()

if __name__ == '__main__':
    #Input the number of board images to use for calibration (recommended: ~20)
    n_boards = 10
    #Input the number of squares on the board (width and height)
    board_w = 9
    board_h = 6
    #Board dimensions (typically in cm)
    board_dim = 25
    #Image resolution
    image_size = (864, 480)

    #Crop mask 
    # A value of 0 will crop out all the black pixels.  This will result in a loss of some actual pixels.
    # A value of 1 will leave in all the pixels.  This maybe useful if there is some important information 
    # at the corners.  Ideally, you will have to tweak this to see what works for you.
    crop = 0


    value = input("Do you want to define your calibration's images ?(y:1/n:0):\n")
    if value == 1:
        save_image()
        calibration(n_boards, board_w, board_h, board_dim)
    elif value == 0:
        calibration(n_boards, board_w, board_h, board_dim)
    else:
        print("Wrong value --> exit program")
        exit()