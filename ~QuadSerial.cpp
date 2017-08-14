#include "QuadSerial.h"
#define T_CHANNEL_MIN	1200
#define T_CHANNEL_MAX	1800
void QuadSerial::writeSyma(float toSend[4], bool picture, bool video)
{
//	if (!result) return;
			//convert 	  Range between LadyBird and syma
	unsigned char pitch = (255* ((toSend[0] - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN)))+0.5;
	if (pitch < 128)
		pitch = 128 - pitch;
	unsigned char roll = (255 * ((toSend[1] - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN)))+0.5;
	if (roll < 128)
		roll = 128 - roll;
	unsigned char throttle = 255 - 255 * ((toSend[2] - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN));
	unsigned char yaw = (255 * ((toSend[3] - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN)))+0.5;
	if (yaw < 128)
		yaw = 128 - yaw;
	//unsigned char yaw = 255-(255 * ((toSend[3] - CHANNEL_MIN) / (CHANNEL_MAX - CHANNEL_MIN))) ;

	//Send 128,128,7,128
	roll = pitch = yaw =128;
	throttle = 7;
	symaBuffer[0] = 255;
	symaBuffer[1] = roll;
	symaBuffer[2] = pitch;
	symaBuffer[3] = throttle;
	symaBuffer[4] = yaw;
	symaBuffer[5] = 0;	 //	 flip
	symaBuffer[6] = picture ? 150 : 0;	//picture
	symaBuffer[7] = video ? 150 : 0;	//video
//	serial.WriteData((char*) symaBuffer, 8);
	printf("Send %d,%d,%d,%d\n", roll, pitch, throttle, yaw);
	int len=1024;
	//serial.ReadData((char*)dummyBuffer, len);
	
	serial2->write((char*)symaBuffer, 8);

	char local[1000];
	serial2->read(local, 1000);
	

}