#include "TinyGPS++.h"
#include "MPU9250_Raw.h"
#include "machine.h"
#include "SoftwareSerial.h"

SoftwareSerial Serial_GPS(SOFTWEAR_SERIAL_PIN_TX, SOFTWEAR_SERIAL_PIN_RX);
TinyGPSPlus GPS;
MPU9250_Raw MPU;

void Machine::Initialize(){
	Serial_GPS.begin(9600);
	MPU.Initialize();
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
	float H[3],A[3],n[3];
	//------------------------------------
}

float AccMagCaluration(float *Acc, float *Mag){
	float n[3],theta,H[3];
	theta = atan(sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1]) / Acc[2]);
	n[0] = -Acc[1] / sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1]);
	n[1] = Acc[0] / sqrt(Acc[0] * Acc[0] + Acc[1] * Acc[1]);
	n[2] = 0;
	H[0] = Mag[0] * (n[0] * n[0] * (1 - cos(-theta)) + cos(-theta) + Mag[1] * (n[0] * n[1] * (1 - cos(-theta))) + Mag[2] * n[1] * sin(theta));
}
