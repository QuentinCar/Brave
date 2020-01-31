#!/usr/bin/env python
import cv2
from time import time
import socket
from goprocam import GoProCamera, constants

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


gpCam = GoProCamera.GoPro()
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
t=time()
gpCam.livestream("start")
gpCam.video_settings(res='1080p', fps='30')
gpCam.gpControlSet(constants.Stream.WINDOW_SIZE, constants.Stream.WindowSize.R720)
cap = cv2.VideoCapture("udp://10.5.5.9:8554", cv2.CAP_FFMPEG)
#!/usr/bin/env python
image_pub = rospy.Publisher("GoPro",Image,queue_size=10)
rospy.init_node('GoPro_Node', anonymous=True)
rate = rospy.Rate(10) # 10hz
#!/usr/bin/env python3

while not rospy.is_shutdown():
    nmat, frame = cap.read()
    cv2.imshow("GoPro OpenCV", frame)

    #!/usr/bin/env python
    image = CvBridge().cv2_to_imgmsg(frame, "bgr8")
    image_pub.publish(image, "RGB8") #conversion OPENCV -> Ros Image Publisher + publish

    #!/usr/bin/env python3
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if time() - t >= 2.5:
        sock.sendto("_GPHD_:0:0:2:0.000000\n".encode(), ("10.5.5.9", 8554))
        t=time()



# When everything is done, release the capture
cap.release()
cv2.destroyAllWindows()
