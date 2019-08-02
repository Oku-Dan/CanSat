#include "TinyGPS++.h"
#include "MPU9250_Raw.h"
#include "machine.h"
#include "SoftwareSerial.h"

SoftwareSerial Serial_GPS(SOFTWEAR_SERIAL_PIN_TX, SOFTWEAR_SERIAL_PIN_RX);
TinyGPSPlus GPS;
MPU9250_Raw MPU;

float AccMagCaluration(float* Acc, float* Mag, int Declination)
{
	float n[3], theta, H[2];
	theta = atan2(sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1]), Acc[2]);
	n[0] = -Acc[1] / sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1]);
	n[1] = Acc[0] / sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1]);
	n[2] = 0;
	H[0] = Mag[0] * (n[0] * n[0] * (1 - cos(-theta)) + cos(-theta) + Mag[1] * (n[0] * n[1] * (1 - cos(-theta))) + Mag[2] * n[1] * sin(theta));
	H[1] = Mag[0] * (n[0] * n[1] * (1 - cos(theta))) + Mag[1] * (n[1] * n[1]*(1 - cos(-theta)) + cos(-theta)) - Mag[2] * n[1] * sin(theta);
	//H[2] = -Mag[0] * n[1] * sin(theta) + Mag[1] * n[0] * sin(theta) + Mag[2] * cos(theta);
	return -(((int)(atan2(H[1], H[0]) * 57.3) /*180 / 3.14*/ + 270 - Declination) % 360 - 360);
}

void Machine::Initialize()
{
	Serial_GPS.begin(9600);
	MPU.Initialize();
	declination = 8;	//仙台周辺
}
void Machine::Update(){
	//-----------GPS----------------------
	while (Serial_GPS.available() > 0)
	{
		GPS.encode(Serial_GPS.read());
		if (GPS.location.isUpdated())
		{
			if(is_GPS_available = true)
			lat = GPS.location.lat();
			lon = GPS.location.lng();
		}
	}
	//------------------------------------

	//-----------Direction----------------
	MPU.UpDate();
	float Mag[3],Acc[3];
	Mag[0] = MPU.magX;
	Mag[1] = MPU.magY;
	Mag[2] = MPU.magZ;
	Acc[0] = MPU.accX;
	Acc[1] = MPU.accY;
	Acc[2] = MPU.accZ;
	direction_world = AccMagCaluration(Mag, Acc, declination);
	//------------------------------------
}
