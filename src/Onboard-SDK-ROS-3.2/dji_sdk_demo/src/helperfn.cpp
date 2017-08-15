/**
 * @brief Helper Function Set
 */

#include "helperfn.hpp"

float this_angle = 0;

std::string stringCat(std::string a, std::string b)
{
    return a + b;
}

float outSClear(float num, float limit)
{
    if(num < -limit || num > limit)
    {
        return 0;
    }
    return num;
}

float outClear(float num, float lower, float upper)
{
    if(num < lower || num > upper)
    {
        return 0;
    }
    return num;
}

float inSRange(float num, float limit)
{
	if (num > limit)
	{
		return limit;
	}
	else if (num < -limit)
	{
		return -limit;
	}
	else
		return num;
}

float inRange(float num, float lower, float upper)
{
	if (num > upper)
	{
		return upper;
	}
	else if (num < lower)
	{
		return lower;
	}
	else
		return num;
}

void originTranslation(CtrlType::Vector3& dest, const CtrlType::Vector3& src, bool isReversed=false)
{
    if(isReversed)
    {
        dest.x = dest.x + src.y;
        dest.y = dest.y + src.x;
        dest.z = -dest.z + src.z;
    }
    else
    {
        dest.x = dest.x + src.x;
        dest.y = dest.y + src.y;
        dest.z = dest.z + src.z;
    }
}

void genVector3Msg(std::stringstream& msg, std::string pre, \
					float x, float y, float z)
{
	msg << pre <<" {";
    msg << x << ",\t";
    msg << y << ",\t";
    msg << z << " }";
}

void LoadParamFile(CtrlType::Vector3& expect, CtrlType::Vector3& origin, \
                CtrlType::dpid_T &pid_x, CtrlType::dpid_T& pid_y, CtrlType::pid_T &pid_z, bool resetPID)
{

    try{
        cv::FileStorage fs(stringCat(LOGFILE_ROOT, "flight_new_param.yaml"), cv::FileStorage::READ);
        std::cout << stringCat(LOGFILE_ROOT, "flight_new_param.yaml") << std::endl;

        fs["ROTATE_ANGLE"] >> this_angle;

        auto itD = fs["DEST"].begin(), itV = fs["VEL_LIM"].begin(), itE = fs["ERR_LIM"].begin();
        auto itX = fs["PID_X"].begin(), itY = fs["PID_Y"].begin(), itZ = fs["PID_Z"].begin();
        auto itO = fs["Origin"].begin();
        //n.type() == FileNode::SEQ

        expect.x = *(itD); expect.y = *(++itD); expect.z = *(++itD);
        origin.x = *(itO); origin.y = *(++itO); origin.z = *(++itO);
        pid_x.pos_loop.vel_lim = (float)(*itV); pid_y.pos_loop.vel_lim = (float)(*++itV); pid_z.vel_lim = (float)(*++itV);
        pid_x.pos_loop.err_lim = (float)(*itE); pid_y.pos_loop.err_lim = (float)(*++itE); pid_z.err_lim = (float)(*++itE);

        if(resetPID == PID_RESET)
        {
            pid_x.pos_loop.kp = (float)(*itX); pid_x.pos_loop.ki = (float)(*++itX); pid_x.pos_loop.kd = (float)(*++itX);
            pid_x.vel_loop.kp = (float)(*++itX); pid_x.vel_loop.ki = (float)(*++itX); pid_x.vel_loop.kd = (float)(*++itX);

            pid_y.pos_loop.kp = (float)(*itY); pid_y.pos_loop.ki = (float)(*++itY); pid_y.pos_loop.kd = (float)(*++itY);
            pid_y.vel_loop.kp = (float)(*++itY); pid_y.vel_loop.ki = (float)(*++itY); pid_y.vel_loop.kd = (float)(*++itY);

            pid_z.kp = (float)(*itZ); pid_z.ki = (float)(*++itZ); pid_z.kd = (float)(*++itZ);
        }

        std::stringstream tmp;
        genVector3Msg(tmp, "PID_X_pos", pid_x.pos_loop.kp, pid_x.pos_loop.ki, pid_x.pos_loop.kd); tmp << std::endl;
        genVector3Msg(tmp, "PID_X_vel", pid_x.vel_loop.kp, pid_x.vel_loop.ki, pid_x.vel_loop.kd); tmp << std::endl;

        genVector3Msg(tmp, "PID_Y_pos", pid_y.pos_loop.kp, pid_y.pos_loop.ki, pid_y.pos_loop.kd); tmp << std::endl;
        genVector3Msg(tmp, "PID_Y_vel", pid_y.vel_loop.kp, pid_y.vel_loop.ki, pid_y.vel_loop.kd); tmp << std::endl;

        genVector3Msg(tmp, "PID_Z", pid_z.kp, pid_z.ki, pid_z.kd);

        std::cout << tmp.str() << std::endl;
        std::cout << "Successful" <<std::endl;
    }
    catch (...) {
        std::cout << "hahahahahah" <<std::endl;
        return;
    }

}

void logToFile(const std::stringstream& msg, int type)
{
    std::time_t result = std::time(nullptr);
    std::fstream fs;

    switch(type)
    {
    case GUIDANCE_MSG:
        fs.open(stringCat(LOGFILE_ROOT, "guidance.txt"), std::fstream::out | std::fstream::app);
        fs << "["<< result <<"] ";
        fs << "[GUIDANCE]";
        break;
    case RC_MSG:
        fs.open(stringCat(LOGFILE_ROOT, "controller.txt"), std::fstream::out | std::fstream::app);
        fs << "["<< result <<"] ";
        fs << "[RC]";
        break;
    case CLIENT_MSG:
        fs.open(stringCat(LOGFILE_ROOT, "client.txt"), std::fstream::out | std::fstream::app);
        fs << "["<< result <<"] ";
        fs << "[CLIENT]";
        break;
    }

    fs<<msg.str() <<std::endl;
    fs.flush();
    fs.close();
}

void resetLogFileAll()
{
    remove(stringCat(LOGFILE_ROOT, "guidance.txt").c_str());
    remove(stringCat(LOGFILE_ROOT, "controller.txt").c_str());
    remove(stringCat(LOGFILE_ROOT, "client.txt").c_str());
}


