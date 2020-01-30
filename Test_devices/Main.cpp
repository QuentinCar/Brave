#include "RazorAHRS.h"
#include "ublox.h"
#include "Pololu.h"
#include "SSC32.h"
#include "IM483I.h"

// Comment/uncomment lines depending on the device you wish to test.
// Change the device path and other parameters in the configuration files if necessary.
// If you are using an IDE, check that the configuration files are in the correct folder for that IDE 
// (e.g. sometimes in the generated ../Test_devices-build-desktop folder for Qt).

int main(int argc, char* argv[]) 
{
	RAZORAHRS razorahrs;
	RAZORAHRSDATA razorahrsdata;
	NMEADATA nmeadata;
	UBLOX ublox;
	double value = 0;
	POLOLU pololu;
	SSC32 ssc32;
	IM483I im483i;
	double u1 = 0.25, u2 = -0.25;

	// Initialize to 0 all the fields of the structure.
	memset(&razorahrs, 0, sizeof(RAZORAHRS));
	memset(&ublox, 0, sizeof(UBLOX));
	memset(&pololu, 0, sizeof(POLOLU));
	memset(&ssc32, 0, sizeof(SSC32));
	memset(&im483i, 0, sizeof(IM483I));

	ConnectRazorAHRS(&razorahrs, "RazorAHRS0.txt"); //IMU Razor
	Connectublox(&ublox, "ublox0.txt"); // Furino Station Meteo
	ConnectPololu(&pololu, "Pololu0.txt");
	//ConnectSSC32(&ssc32, "SSC320.txt"); // Servo Controler Rudder
	//ConnectIM483I(&im483i, "IM483I0.txt"); // Servo Controler Winch

	
	// Wait a little bit...
	mSleep(500);
	//mSleep(20000);

	GetLatestDataRazorAHRS(&razorahrs, &razorahrsdata);
	printf("%f %f %f\n", razorahrsdata.Yaw*180.0/M_PI, razorahrsdata.Pitch*180.0/M_PI, razorahrsdata.Roll*180.0/M_PI);

	GetNMEASentenceublox(&ublox, &nmeadata);
	printf("%f;%f\n", nmeadata.Latitude, nmeadata.Longitude);

	u1 = u2;
	u2 = -u1;
	SetRudderThrustersPololu(&pololu, u1, u2, u2);
	//SetRudderThrusterSSC32(&ssc32, u1, u2);
	//SetMaxAngleIM483I(&im483i, u1);
	printf("%f;%f\n", u1, u2);

	//DisconnectIM483I(&im483i);
	//DisconnectSSC32(&ssc32);
	DisconnectPololu(&pololu);
	Disconnectublox(&ublox);
	DisconnectRazorAHRS(&razorahrs);

	return EXIT_SUCCESS;
}
