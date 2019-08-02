#ifndef MACHINE_H
#define MACHINE_H

#define SOFTWEAR_SERIAL_PIN_RX 2
#define SOFTWEAR_SERIAL_PIN_TX	3

class Machine
{
	public :
		int direction_world;	//北を0度とした時計回りの方角
		int declination;
		float lat;	//緯度
		float lon;	//経度
		bool is_GPS_available = false;
		void Update();
		void Initialize();

	private:
};

#endif
