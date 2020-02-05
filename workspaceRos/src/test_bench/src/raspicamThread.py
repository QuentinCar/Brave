#!/usr/bin/env python
# -*- coding: utf-8 -*-

####### USING https://www.pyimagesearch.com/2015/12/28/increasing-raspberry-pi-fps-with-python-and-opencv/ ###########


# import the necessary packages
import rospy
import rospkg
from picamera.array import PiRGBArray
from picamera import PiCamera
from threading import Thread
import cv2
import time
from numpy import array, tan
# from sensor_msgs.msg import Image, CameraInfo
# from cv_bridge import CvBridge, CvBridgeError

from chessboard_calibration import getCamDistortData



class PiVideoStream:
    def __init__(self, resolution=(640, 480), framerate=15, mode = 'sports', record = False):
        r = rospkg.RosPack()
        package_path = r.get_path('test_bench')

        # initialize the camera and stream
        self.camera = PiCamera()
        self.camera.resolution = resolution
        self.camera.framerate = framerate
        self.camera.exposure_mode = mode
        self.scale_factor = tan(0.398)/(resolution[0]*203.55/640)   #Camera scale factor, rad/pixel

        self.rawCapture = PiRGBArray(self.camera, size=resolution)
        self.stream = self.camera.capture_continuous(self.rawCapture,format="bgr", use_video_port=True)

        # self.image_pub = rospy.Publisher("camera_rect/image_rect",Image, queue_size = 0)
        # self.info_pub = rospy.Publisher("camera_rect/camera_info",CameraInfo, queue_size = 0)
        # self.bridge = CvBridge()
        # self.camInfo = CameraInfo()

        # Read the camera matrix from calibration file
        self.calibration_matrix, self.calibration_dist = getCamDistortData(package_path+'/src/calibration_data.txt')

        # initialize the frame and the variable used to indicate
        # if the thread should be stopped
        self.frame = None
        self.stopped = False

        # allow the camera to warmup
        time.sleep(0.1)

        self.record = record

        if record:
            # Define the codec and create VideoWriter object
            self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
            self.out = cv2.VideoWriter(package_path+'/missionRecord/Mission_'+time.strftime('%F')+'_'+time.strftime('%a')+'_'+time.strftime('%H')+'_'+time.strftime('%M')+'(without calibration).avi',self.fourcc, framerate, resolution)


    def start(self):
        # start the thread to read frames from the video stream
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        # keep looping infinitely until the thread is stopped
        for f in self.stream:
            if self.record:
                self.out.write(f.array)

            # grab the frame from the stream and clear the stream in
            # preparation for the next frame
            self.frame = cv2.undistort(f.array, self.calibration_matrix, self.calibration_dist, None)
            self.rawCapture.truncate(0)

            # try:
            #     self.image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
            #     self.info_pub.publish(self.camInfo)
            # except Exception as e:
            #     rospy.loginfo('Error cam: {0}'.format(e))

            # if the thread indicator variable is set, stop the thread
            # and resource camera resources
            if self.stopped:
                self.stream.close()
                self.rawCapture.close()
                self.camera.close()
                if self.record:
                    self.out.release()
                return

    def read(self):
        # return the frame most recently read
        return self.frame

    def stop(self):
        # indicate that the thread should be stopped
        self.stopped = True

    def getScaleFactor(self):
        return (self.scale_factor, self.camera.resolution)




if __name__ == "__main__":
    rospy.init_node('image_visualisation', anonymous=True)

    rate = rospy.Rate(5)

    vs = PiVideoStream(resolution=(320, 240), framerate=5, mode = 'sports', record = True).start()

    time.sleep(2)

    dataCam = vs.getScaleFactor()


    while not rospy.is_shutdown():
        image = vs.read()
        cv2.imshow("Test", image)

        key = cv2.waitKey(1) & 0xFF
        if key == 27 or key == ord('q'):
                break

        elif key == 32:
            key = cv2.waitKey(1) & 0xFF
            while key != 32:
                key = cv2.waitKey(1) & 0xFF
        
        rate.sleep()


    vs.stop()
    cv2.destroyAllWindows()