import numpy as np
import cv2
import glob



def getCamDistortData(filename):
    saveFile = open(filename, 'r')
    lines = saveFile.readlines()
    saveFile.close()

    for i in range(len(lines)):
        if lines[i] == '#mtx\n':
            i+=1
            mtxString = lines[i]+lines[i+1]+lines[i+2]
            mtxList = mtxString.split()
            mtx = np.zeros((3,3))
            cnt = 0

            for part in mtxList:

                try:
                    while part[0] == '[':
                        part = part[1:]

                    while part[-1] == ']':
                        part = part[:-1]

                    mtx[cnt//3][cnt%3] = float(part)
                    cnt+=1
                except:
                    pass

        if lines[i] == '#dist\n':
            i+=1
            distString = lines[i]+lines[i+1]
            distList = distString.split()
            dist = np.zeros((1,5))
            cnt = 0

            for part in distList:

                try:
                    while part[0] == '[':
                        part = part[1:]

                    while part[-1] == ']':
                        part = part[:-1]

                    dist[0][cnt] = float(part)
                    cnt+=1
                except:
                    pass

            return mtx, dist





if __name__ == "__main__":
    # termination criteria
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*9,3), np.float32)
    objp[:,:2] = np.mgrid[0:9,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    images = glob.glob('calibration_images/*.png')
    count = 0

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, (9,6),None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            count += 1
            objpoints.append(objp)

            corners2 = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            img = cv2.drawChessboardCorners(img, (9,6), corners2,ret)
            cv2.imshow('img',img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()

    print("Found ", count, "correct images.")

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1],None,None)

    saveFile = open("calibration_data.txt", 'w')
    saveFile.write("#ret" +'\n'+ str(ret) +'\n'+ '#mtx'+'\n'+ str(mtx) +'\n'+ '#dist'+'\n'+ str(dist) +'\n'+ '#rvecs'+'\n'+ str(rvecs) +'\n'+ '#tvecs'+'\n'+str(tvecs))
    saveFile.close()


    img = cv2.imread('calibration_images/fra35.png')
    h,  w = img.shape[:2]


    # undistort
    dst = cv2.undistort(img, mtx, dist, None)


    cv2.imwrite('calibration_images/calibresult_fra35.png',dst)






