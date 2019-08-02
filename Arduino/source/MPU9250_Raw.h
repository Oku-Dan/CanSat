#include <Wire.h>
#include "Arduino.h"

#define MPU9250_ADDRESS 0x68 //I2CでのMPU9250のスレーブアドレス
#define PWR_MGMT_1 0x6b		 //電源管理のアドレス，スリープモード解除用
#define INT_PIN_CFG 0x37	 //磁気センサのバイパスモード設定用のアドレス

#define ACCEL_CONFIG 0x1c	//加速度センサ設定用のアドレス
#define ACCEL_FS_SEL_2G 0x00 //加速度センサのレンジ(2G)
#define ACCEL_FS_SEL_4G 0x08 //加速度センサのレンジ(4G)
#define ACCEL_FS_SEL_8G 0x10 //加速度センサのレンジ(8G)
#define ACCEL_FS_SEL_16G 0x18	//加速度センサのレンジ(16G)

#define GYRO_CONFIG 0x1b		 //ジャイロセンサ設定用のアドレス
#define GYRO_FS_SEL_250DPS 0x00  //ジャイロセンサのレンジ(250DPS)
#define GYRO_FS_SEL_500DPS 0x08  //ジャイロセンサのレンジ(500DPS)
#define GYRO_FS_SEL_1000DPS 0x10 //ジャイロセンサのレンジ(1000DPS)
#define GYRO_FS_SEL_2000DPS 0x18 //ジャイロセンサのレンジ(2000DPS)

#define AK8963_ADDRESS 0x0c		  //磁気センサのスレーブアドレス
#define CNTL1 0x0a				  //磁気センサ設定用のアドレス
#define CNTL1_MODE_SEL_8HZ 0x12   //磁気センサの出力周期(8Hz)
#define CNTL1_MODE_SEL_100HZ 0x16 //磁気センサの出力周期(100Hz)
#define ST1 0x02 //データ読み込み用フラッグのアドレス

#define SAMPLING_RATE 100 //Hz

float accRange;  //計算で使用するので，選択したレンジを入力する定数
float gyroRange; //計算で使用するので，選択したレンジを入力する定数
float magRange; //計算で使用するので，選択したレンジを入力する定数

uint8_t AccelGyroData[14]; //センサからのデータ格納用配列
uint8_t MagneticData[7];	 //センサからのデータ格納用配列
uint8_t ST1Bit;			  //磁気センサのフラッグ

void I2CRead(uint8_t Address, uint8_t Register, uint8_t BytesToRead, volatile uint8_t *Data)
{
	Wire.beginTransmission(Address); //指定したアドレスと通信を始める
	Wire.write(Register);			 //レジスタを書き込む
	Wire.endTransmission();			 //通信を終了する

	Wire.requestFrom(Address, BytesToRead); //スレーブからBytesToReadのデータを要求する
	uint8_t index = 0;
	while (Wire.available())
	{
		Data[index++] = Wire.read(); //データを読み込む
	}
}

void I2CWrite(uint8_t Address, uint8_t Register, volatile uint8_t Data)
{
	Wire.beginTransmission(Address); //指定したアドレスと通信を始める
	Wire.write(Register);			 //指定するレジスタを書き込む
	Wire.write(Data);				 //データを書き込む
	Wire.endTransmission();			 //通信を終了する
}

class MPU9250_Raw
{
private:
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	int16_t mx = 0;
	int16_t my = 0;
	int16_t mz = 0;
	int16_t t = 0;
	unsigned int interval = 0;

public:
	float accX = 0; //加速度センサから求めた重力加速度
	float accY = 0; //加速度センサから求めた重力加速度
	float accZ = 0; //加速度センサから求めた重力加速度
	float gyroX = 0; //ジャイロセンサから求めた角速度
	float gyroY = 0; //ジャイロセンサから求めた角速度
	float gyroZ = 0; //ジャイロセンサから求めた角速度
	float magX = 0; //磁気センサから求めたuT
	float magY = 0; //磁気センサから求めたuT
	float magZ = 0; //磁気センサから求めたuT
	float temp = 0; //MPU9250の温度
	float mag_offset_x = 0;	//磁気センサのオフセット
	float mag_offset_y = 0; //磁気センサのオフセット
	float mag_offset_z = 0; //磁気センサのオフセット
	unsigned long last_samplingTime = 0;

	void Initialize(){
		I2CWrite(MPU9250_ADDRESS, PWR_MGMT_1, 0x00);				 //スリープモードを解除
		I2CWrite(MPU9250_ADDRESS, ACCEL_CONFIG, ACCEL_FS_SEL_16G);   //加速度センサの測定レンジの設定
		accRange = 16.0 / 32768.0;									 //計算で使用するので，選択したレンジを入力する
		I2CWrite(MPU9250_ADDRESS, GYRO_CONFIG, GYRO_FS_SEL_2000DPS); //ジャイロセンサの測定レンジの設定
		gyroRange = 2000.0 / 32768.0;								 //計算で使用するので，選択したレンジを入力する
		I2CWrite(MPU9250_ADDRESS, INT_PIN_CFG, 0x02);				 //bypass mode(磁気センサが使用出来るようになる)
		I2CWrite(AK8963_ADDRESS, CNTL1, CNTL1_MODE_SEL_100HZ);		 //磁気センサのAD変換開始(0x16:100Hz 0x12:8Hz)
		magRange = 4921.0f * 10.0f / 32768.0f;
	}

	bool UpDate(){
		interval = micros() - last_samplingTime;
		if(interval  < 1000000 / SAMPLING_RATE){
			return false;
		}
		else{
			last_samplingTime += interval;
		}
		I2CRead(MPU9250_ADDRESS, 0x3b, 14, AccelGyroData); //0x3bから，14バイト分をaccGyroDataにいれる
		I2CRead(AK8963_ADDRESS, ST1, 1, &ST1Bit);	//磁気センサのフラグチェック
		if ((ST1Bit & 0x01))
		{
			I2CRead(AK8963_ADDRESS, 0x03, 7, MagneticData); //7番目の0x09(ST2)まで読まないとデータが更新されない
		}

		ax = (AccelGyroData[0] << 8) | AccelGyroData[1];
		ay = (AccelGyroData[2] << 8) | AccelGyroData[3];
		az = (AccelGyroData[4] << 8) | AccelGyroData[5];

		t = (AccelGyroData[6] << 8) | AccelGyroData[7];

		gx = (AccelGyroData[8] << 8) | AccelGyroData[9];
		gy = (AccelGyroData[10] << 8) | AccelGyroData[11];
		gz = (AccelGyroData[12] << 8) | AccelGyroData[13];

		mx = (MagneticData[3] << 8) | MagneticData[2];	//センサの軸が違うので順番が加速度とジャイロと違う
		my = (MagneticData[1] << 8) | MagneticData[0];
		mz = -((MagneticData[5] << 8) | MagneticData[4]); //加速度，ジャイロセンサと軸の向きが逆なので-を掛ける

		accX = ax * accRange; //[G]
		accY = ay * accRange;
		accZ = az * accRange;

		gyroX = gx * gyroRange; //[deg/s]
		gyroY = gy * gyroRange;
		gyroZ = gz * gyroRange;

		temp = ((t - 0.0) / 333.87) + 21.0f;

		magX = (mx - mag_offset_x) * magRange; //[mGause]
		magY = (my - mag_offset_y) * magRange;
		magZ = (mz - mag_offset_z) * magRange;

		return true;
	}
};
