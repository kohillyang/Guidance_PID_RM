/*
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
*/
#ifndef __TYPE_H
#define __TYPE_H
#include <stdio.h>
#include <string>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <math.h>
#include <ros/ros.h>
#include <iostream>
using namespace std;
#define RECIEVE_BUF_SIZE 24

#define Byte16(Type, ByteH, ByteL)  ((Type)((((unsigned short int)(ByteH))<<8) | ((unsigned short int)(ByteL))))
typedef struct
{
	float X;
	float Y;
	float Z;
}S_FLOAT_XYZ;

typedef struct
{
	signed short int X;
	signed short int Y;
	signed short int Z;
}S_INT16_XYZ;//s16

typedef struct
{
	signed int X;
	signed int Y;
	signed int Z;
}S_INT32_XYZ;//s32

int init_pm25(const char *device, int baudrate);
int read_pm25(unsigned char *buf, int len);
void close_pm25();
void INF_DW1000_Module_Data_Convert(unsigned char recieve_buf[RECIEVE_BUF_SIZE],float *uwb_position_x,float *uwb_position_y,float *uwb_position_z);

#endif
