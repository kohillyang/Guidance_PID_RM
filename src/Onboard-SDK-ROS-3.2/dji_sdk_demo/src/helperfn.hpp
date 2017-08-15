
#ifndef __HELPERFN_HPP__
#define __HELPERFN_HPP__

#include <ctime>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "pid_set.h"

#define LOGFILE_ROOT	"/home/ubuntu/logFile/"
#define GUIDANCE_MSG	0x01
#define RC_MSG			0x02
#define CLIENT_MSG		0x03

#define PID_KEEP		0x00
#define PID_RESET		0x01

#define LOGFILE_RESET	0x00
#define LOGFILE_SAVE	0x01

float outSClear(float num, float limit);
float outClear(float num, float lower, float upper);

float inSRange(float num, float limit);
float inRange(float num, float lower, float upper);

void originTranslation(CtrlType::Vector3& dest, const CtrlType::Vector3& src, bool isReversed);

void genVector3Msg(std::stringstream& msg, std::string pre, \
					float x, float y, float z);

void resetLogFileAll(void);
void logToFile(const std::stringstream& msg, int type);

void LoadParamFile(CtrlType::Vector3& expect, CtrlType::Vector3 &origin, \
                CtrlType::dpid_T& pid_x, CtrlType::dpid_T &pid_y, CtrlType::pid_T &pid_z, bool resetPID);

#endif
