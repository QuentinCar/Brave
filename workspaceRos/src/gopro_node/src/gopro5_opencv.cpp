// OpenCV headers.
#include "opencv2/highgui.hpp"
#include <cv.h>

//Socket headers
#include <stdio.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <unistd.h> 
#include <string.h>


// Need to send first : http://10.5.5.9/gp/gpControl/execute?p1=gpStream&a1=proto_v2&c1=restart
// See GoProStream.py
// Keep-alive command : "_GPHD_:0:0:2:0.000000"

int main(int argc, char* argv[])
{
	// Will store the code of the keyboard key pressed to terminate the program.
	int c = 0;

	// Keep-alive...
	int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	struct sockaddr_in sa;
	memset(&sa, 0, sizeof(sa));
	sa.sin_family = AF_INET;
	sa.sin_port = htons((unsigned short)atoi("8554"));
	sa.sin_addr.s_addr = inet_addr("10.5.5.9");
	int salen = sizeof(sa);
	
	cv::VideoCapture webcam0 = cv::VideoCapture();

	// Open camera number 0.
	webcam0.open("udp://10.5.5.9:8554", cv::CAP_FFMPEG);

	// Set desired camera resolution. Depending on the camera, some settings might not be available.
	webcam0.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
	webcam0.set(CV_CAP_PROP_FRAME_HEIGHT, 720);

	// Will correspond to the current image of the camera.
	cv::Mat image = cv::Mat();

	// Creation of a window entitled "Webcam".
	cv::namedWindow("Webcam");

	for (;;)
	{
		// Get the current image from the camera.
		webcam0.read(image);

		// Display it in the window.
		cv::imshow("Webcam", image);

		// Keep-alive...
		sendto(sock, "_GPHD_:0:0:2:0.000000", sizeof("_GPHD_:0:0:2:0.000000"), 0, (struct sockaddr*)&sa, salen);

		// Wait up to 10 ms for the user to press a key, and return the key code.
		// This function is also necessary to refresh the window content.
		c = cv::waitKey(10);

		// Stop the loop if the ESC key was pressed (ASCII code).
		if ((char)c == 27)
		{
			break;
		}
	}

	// Free the memory used by the camera.
	webcam0.release();

	// Destroy the window previously created.
	cv::destroyWindow("Webcam");

	return 0;
}
