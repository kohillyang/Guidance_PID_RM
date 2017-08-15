  /** @file controllerNode.cpp
 *  @version 3.1.8
 *  @date July 29th, 2016
 *
 *  @brief
 *  controllerNode are implemented here. 
 *
 *  @copyright 2016 DJI. All rights reserved.
 *
 */
#ifdef __cplusplus__
  #include <cstdlib>
#else
  #include <stdlib.h>
#endif

#define RESET_TO_DEFAULT    0
#define RESET_TO_FILE       1
#include <signal.h>
#include <thread>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include <dji_sdk_demo/StatusCodeStamped.h>
#include <opencv2/opencv.hpp>

#include "ctrl_type.h"
#include "controllerNode.h"
#include "gpio_set.h"

static bool waitFlag = 0, interrupted = 1;
static ros::Publisher *position_pub;
static enum FlightStatus drone_status;

dji_sdk_controller::flight_msg pid_px, pid_py, pid_z;
dji_sdk_controller::flight_msg pid_vx, pid_vy;
dji_sdk_controller::flight_msg dest, vel_limit, err_limit;
dji_sdk_controller::flight_msg car_pose, circle_pose, tape_pose;
bool car_pose_detected, circle_pose_detected, tape_pose_detected;

bool command_process(int layer, std::istream& input, std::string param);

void catch_signal(int sign)
{
    switch (sign)
    {
    case SIGINT:
        waitFlag = 0;
        interrupted = 1;
        std::cout << "Interrupted!\n" << std::endl;
        break;
    default:
        break;
    }
}

void drone_status_callback(const dji_sdk_demo::StatusCodeStamped& msg)
{
   drone_status = static_cast<enum FlightStatus>(msg.code.data);
}

void circle_detect_callback(const dji_sdk_controller::flight_msg& msg)
{
    circle_pose = msg;
    circle_pose_detected = true;
}

void car_detect_callback(const dji_sdk_controller::flight_msg& msg)
{
    car_pose = msg;
    car_pose_detected = true;
}

void tape_detect_callback(const dji_sdk_controller::flight_msg& msg)
{
    tape_pose = msg;
    tape_pose_detected = true;
}

void genVector3Msg(std::stringstream& msg, std::string pre, \
                    float x, float y, float z)
{
    msg << pre <<" (";
    msg << x << ",\t";
    msg << y << ",\t";
    msg << z << " )";
}

void disp_helper()
{
    std::cout << "            Remote Controller Helper             " <<std::endl;
    std::cout << " =============================================== " <<std::endl;
    std::cout << "   [TAKEOFF] Takeoff                             " <<std::endl;
    std::cout << "   [L/Landing] Landing                           " <<std::endl;
    std::cout << "   [D] Destination Coordinate Set                " <<std::endl;
    std::cout << "   [INC] Incremental Destination Coordinate      " <<std::endl;
    std::cout << "   [STATIC] World Destination Coordinate Set     " <<std::endl;
    std::cout << "   [XP] PID set for x-axis position              " <<std::endl;
    std::cout << "   [YP] PID set for x-axis position              " <<std::endl;
    std::cout << "   [PZ] PID set for x-axis position              " <<std::endl;
    std::cout << "   [XV] PID set for x-axis velocity              " <<std::endl;
    std::cout << "   [YV] PID set for x-axis velocity              " <<std::endl;
    std::cout << "   [T/DETECT] Detection position send            " <<std::endl;
    std::cout << "   [STATE] Init State Machine 		           " <<std::endl;
    std::cout << "   	Param: INIT=0, GRAB=1, DROP=2, PATROL=5    " <<std::endl;
    std::cout << "   	Param: CIRCLE=3, CAR=4				       " <<std::endl;
    std::cout << "   [WAIT] Wait for drone movement stop           " <<std::endl;
    std::cout << "   [MG] Grab Motor (GPIO157)                     " <<std::endl;
    std::cout << "   [MR] Release Motor (GPIO158)                  " <<std::endl;
    std::cout << "   [RF] Reset to File                            " <<std::endl;
    std::cout << "   [P/Print] Print current parameters.           " <<std::endl;
    std::cout << "   [Clear/Cls] Clear Screen                      " <<std::endl;
    std::cout << "   [Q/Exit] Quit                                 " <<std::endl;
    std::cout << "Please Select One:" <<std::endl;
}

void disp_param()
{
    std::cout << std::endl;
    std::cout << "               Current Parameter Set             " <<std::endl;
    std::cout << " =============================================== " <<std::endl;

    std::cout << " Destination Point: ( " <<dest.data.x<<", " <<dest.data.y<<", " <<dest.data.z<<" )"<<std::endl;

    std::cout << " PID set for X POS:\t{ " <<pid_px.data.x<<"\t" <<pid_px.data.y<<"\t" <<pid_px.data.z<<" }"<<std::endl;
    std::cout << " PID set for X VEL:\t{ " <<pid_vx.data.x<<"\t" <<pid_vx.data.y<<"\t" <<pid_vx.data.z<<" }"<<std::endl;
    std::cout << " PID set for Y POS:\t{ " <<pid_py.data.x<<"\t" <<pid_py.data.y<<"\t" <<pid_py.data.z<<" }"<<std::endl;
    std::cout << " PID set for Y VEL:\t{ " <<pid_vy.data.x<<"\t" <<pid_vy.data.y<<"\t" <<pid_vy.data.z<<" }"<<std::endl;
    std::cout << " PID set for Z:\t{ " <<pid_z.data.x<<"\t" <<pid_z.data.y<<"\t" <<pid_z.data.z<<" }"<<std::endl;

    std::cout << " Velocity limit:\t{ " <<vel_limit.data.x<<"\t" <<vel_limit.data.y<<"\t" <<vel_limit.data.z<<" }"<<std::endl;
    std::cout << " Location Error limit:\t{ " <<err_limit.data.x<<"\t" <<err_limit.data.y<<"\t" <<err_limit.data.z<<" }"<<std::endl;
    std::cout << std::endl;
}

void waitUntilReady(const bool& flag_one=false, const bool& flag_two=false)
{
   int counter = 0;
    ros::Rate r(10);//@10Hz

    ros::Duration(1).sleep();
    ros::spinOnce();
    waitFlag = 1;

    while(waitFlag&&(drone_status==FLIGHT_TAKEOFF_BUSY)&&(!flag_one&&!flag_two))
    {
        ++counter;
        ros::spinOnce();
        r.sleep();
    }
    std::cout << counter << std::endl;
}

void waitUntilReady_proxy(int flag_num)
{
    enum interrupt_flag flag_type = static_cast<enum interrupt_flag>(flag_num);
    switch(flag_type)
    {
    case CIRCLE_DETECTED:
        waitUntilReady(circle_pose_detected);
        break;
    case CAR_DETECTED:
        waitUntilReady(car_pose_detected);
        break;
    case CIRCLE_OR_CAR_DETECTED:
        waitUntilReady(car_pose_detected, circle_pose_detected);
        break;
    default:
        waitUntilReady();
        break;
    }
}

void waitForTime(float seconds)
{
    ros::Duration(seconds).sleep();
}

bool call_proxy(std::string filename, bool basic=false)
{
    bool proxy_callback;
    std::ifstream fin;

    if(basic)
    {
        fin.open(BASIC_SCRIPT_PREFIX + filename);
    }
    else
    {
        fin.open(SCRIPT_PREFIX + filename);
    }
    proxy_callback = command_process(0, fin, "MACHINE");
    fin.close();

    //return proxy_callback;
}

void state_machine(const enum LogicState state)
{
    enum LogicState last_state, this_state = state;

    while(true)//next for some status judgement
    {
        last_state = this_state;

        if(interrupted) this_state = MANUAL_MODE;

        if(car_pose_detected && last_state==PATROL_MODE)
        {
            this_state = BASE_LAND;
        }
        if(circle_pose_detected && last_state==PATROL_MODE)
        {
            this_state = CIRCLE_LAND;
        }

        switch(this_state)
        {
        case INIT:
            //call_proxy("grab.auto");
            this_state = GRAB_MODE;
            break;
        case GRAB_MODE:
            call_proxy("grab.auto");
                call_proxy("takeoff", true);
            this_state = PATROL_MODE;
            break;
        case DROP_MODE:
            call_proxy("drop.auto");
            this_state = CIRCLE_LAND;
            break;
        case CIRCLE_LAND:
            call_proxy("circle_land.auto");
            this_state = GRAB_MODE;
            break;
        case BASE_LAND:
            call_proxy("base_land.auto");
            this_state = DROP_MODE;
            break;
        case PATROL_MODE:
            while(!call_proxy("patrol.auto"));
            break;
        case MANUAL_MODE:
            std::cout << "State Machine Fail!" << std::endl;
            call_proxy("height_fix", true);
            break;
        }
    }

    interrupted = 0;
    return;
}

enum execution_flag command_execute(dji_sdk_controller::flight_msg& msg, std::string cmd, std::istream& input=std::cin)
{
    boost::algorithm::to_upper(cmd);

    if (cmd.compare("L") == 0||cmd.compare("LAND") == 0||cmd.compare("LANDING") == 0)             //0x09
    {
        msg.type = LANDING;
        msg.data.x=0; msg.data.y=0; msg.data.z=0;
        return NO_INPUT_PUB;
    }
    else if (cmd.compare("MG") == 0) // Grab Motor
    {
        setGPIO(GPIO157, GPIO_LOW);
        setGPIO(GPIO157, GPIO_HIGH);
        setGPIO(GPIO157, GPIO_LOW);
        std::cout << "Grab Motor Called.Next: " << std::endl;
        return NO_INPUT_NO_PUB;
    }
    else if (cmd.compare("MR") == 0) // Release Motor
    {
        setGPIO(GPIO158, GPIO_LOW);
        setGPIO(GPIO158, GPIO_HIGH);
        setGPIO(GPIO158, GPIO_LOW);
        std::cout << "Release Motor Called.Next: " << std::endl;
        return NO_INPUT_NO_PUB;
    }
    else if (cmd.compare("TAKEOFF") == 0)                                                         //0x0A
    {
        msg.type = TAKEOFF;
        msg.data.x=0; msg.data.y=0; msg.data.z=0;
        return NO_INPUT_PUB;
    }
    else if (cmd.compare("XP") == 0)             //0x04
    {
        msg.type = PID_X_POS;
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        pid_px.data = msg.data;
        return INPUT_PUB;
    }
    else if(cmd.compare("YP") == 0)         //0x05
    {
        msg.type = PID_Y_POS;
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        pid_py.data = msg.data;
        return INPUT_PUB;
    }
    else if (cmd.compare("XV") == 0)             //0x04
    {
        msg.type = PID_X_VEL;
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        pid_vx.data = msg.data;
        return INPUT_PUB;
    }
    else if(cmd.compare("YV") == 0)         //0x05
    {
        msg.type = PID_Y_VEL;
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        pid_vy.data = msg.data;
        return INPUT_PUB;
    }
    else if(cmd.compare("PZ") == 0)         //0x06
    {
        msg.type = PID_Z;
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        pid_z.data = msg.data;
        return INPUT_PUB;
    }
    else if(cmd.compare("VEL") == 0)        //0x02
    {
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        msg.type = VEL_LIM;
        vel_limit.data = msg.data;
        return INPUT_PUB;
    }
    else if(cmd.compare("ERR") == 0)        //0x03
    {
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        msg.type = ERR_LIM;
        err_limit.data = msg.data;
        return INPUT_PUB;
    }
    else if(cmd.compare("D") == 0||cmd.compare("DEST") == 0)                                      //0x01
    {
        msg.type = DEST;
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        return INPUT_PUB;
    }
    else if(cmd.compare("INC") == 0)                                                              //0x0B
    {
        msg.type = DEST_INC;
        input >> msg.data.x >> msg.data.y >> msg.data.z;// text length or cancel auto-fill
        return INPUT_PUB;
    }
    else if(cmd.compare("STATIC") == 0)                                                           //0x0C
    {
        msg.type = DEST_STATIC;
        input >> msg.data.x >> msg.data.y >> msg.data.z;
        dest.data = msg.data;
        return INPUT_PUB;
    }
    else if(cmd.compare("C") == 0||cmd.compare("C_DETECT") == 0)                                  //0x0B, CIRCLE_DETECT
    {
        if(circle_pose_detected)
        {
            msg.type = DEST_INC;
            msg.data.x = circle_pose.data.x;    msg.data.y = circle_pose.data.y;    msg.data.z = 0;
            circle_pose_detected = false;
            return NO_INPUT_PUB;
        }
        else
        {
            return NO_INPUT_NO_PUB;
        }
    }
    else if(cmd.compare("T") == 0||cmd.compare("T_DETECT") == 0)                                  //0x0B, CAR_DETECT
    {
        if(car_pose_detected)
        {
            msg.type = DEST_INC;
            msg.data.x = car_pose.data.x;    msg.data.y = car_pose.data.y;    msg.data.z = 0;
            car_pose_detected = false;
            return NO_INPUT_PUB;
        }
        else
        {
            interrupted = 1;
            return NO_INPUT_NO_PUB;
        }
    }
    else if(cmd.compare("TAPE_DETECT") == 0)                                                      //0x0B, TAPE_DETECT
    {
        return NO_INPUT_NO_PUB;
    }
    else if(cmd.compare("WAIT_TIME") == 0)                                                        //INTERNAL
    {
        float seconds;
        input >> seconds;
        std::cout << "Wait for time: " << seconds<<std::endl;
        waitForTime(seconds);
        return NO_INPUT_NO_PUB;
    }
    else if(cmd.compare("WAIT") == 0||cmd.compare("WAIT_NONE") == 0)                              //INTERNAL
    {
        waitUntilReady();
        return NO_INPUT_NO_PUB;
    }
    else if(cmd.compare("WAIT_FLAG") == 0)                                                        //INTERNAL
    {
        int flag_type;
        input >> flag_type;
        waitUntilReady_proxy(flag_type);
        return INPUT_NO_PUB;
    }
    else if(cmd.compare("FAIL_SAFE") == 0)                                                        //INTERNAL
    {
        return FAIL_SAFE;
    }
    else if(cmd.compare("SC") == 0||cmd.compare("SCRIPT") == 0)                                   //INTERNAL
    {
        return SCRIPT;
    }
    else if(cmd.compare("P")==0||cmd.compare("PRINT")==0||cmd.compare("DISPLAY")==0)
    {
        system("clear");
        disp_param();   disp_helper();
        return NO_INPUT_NO_PUB;
    }
    else if(cmd.compare("CLEAR")==0||cmd.compare("CLS")==0)
    {
        system("clear");    disp_helper();
        return NO_INPUT_NO_PUB;
    }
    else if(cmd.compare("Q")==0||cmd.compare("EXIT")==0)
    {
        system("clear");
        std::cout << "bye." << std::endl;
        exit(-1);
    }
    else{
        std::cout << "Undeclared Commands." << std::endl << std::endl;
        disp_helper();
        return NO_INPUT_NO_PUB;
    }
}

bool command_process(int layer, std::istream& input, std::string param="SILENT")
{
    std::string cmd;
    dji_sdk_controller::flight_msg msg;
    enum execution_flag cmd_right;

    while(!input.eof()&&!interrupted)//input not meet with endl
    {
        ros::spinOnce();
        input >> cmd;

        cmd_right = command_execute(msg, cmd, input);

        switch(cmd_right)
        {
        case INPUT_PUB:
        case NO_INPUT_PUB:
            position_pub->publish(msg);
            if (param.compare("SILENT") == 0)
            {
                std::stringstream tmp;
                tmp << "[ECHO_"<<layer<<"] ";
                genVector3Msg(tmp, cmd, msg.data.x, msg.data.y, msg.data.z);
                std::cout << tmp.str() << std::endl;
            }
            else{
                std::cout<<std::endl << "[(" <<msg.type<<"), (" <<msg.data.x<<"," <<msg.data.y<<"," <<msg.data.z <<") ] published."<<std::endl;
                std::cout << "[Success!]Please Select Another One:             " <<std::endl;
            }
            break;
        case INPUT_NO_PUB:
        case NO_INPUT_NO_PUB:
            break;
        case FAIL_SAFE:
            break;
        case SCRIPT:
            std::string tmp;
            input >> tmp;

            std::filebuf fbIn;
            fbIn.open(SCRIPT_PREFIX + tmp, std::ios::in);

            waitUntilReady();
            std::istream fin(&fbIn);
            fin >> tmp;
            command_process(layer+1, fin, tmp);
            break;
        }
    }

    if(interrupted)
    {
        std::cout << "[ECHO_"<<layer<<"] INTERRUPTED." << std::endl;
        interrupted = 0;
        return false;//chain effect for next change
    }
    else
    {
        std::cout << "[ECHO_"<<layer<<"] FINISHED." << std::endl<<std::endl;
        return true;
    }
}

int loadFromFile(std::string file_name = "")
{
    if(file_name.empty())
    {
        file_name = "/home/ubuntu/logFile/flight_new_param.yaml";
    }

    try{
        cv::FileStorage fs(file_name, cv::FileStorage::READ);

        auto itD = fs["DEST"].begin(), itV = fs["VEL_LIM"].begin(), itE = fs["ERR_LIM"].begin();
        auto itX = fs["PID_X"].begin(), itY = fs["PID_Y"].begin(), itZ = fs["PID_Z"].begin();
        //n.type() == FileNode::SEQ

        dest.data.x = *(itD); dest.data.y = *(++itD); dest.data.z = *(++itD);

        vel_limit.data.x = (float)(*itV); vel_limit.data.y = (float)(*++itV); vel_limit.data.z = (float)(*++itV);

        err_limit.data.x = (float)(*itE); err_limit.data.y = (float)(*++itE); err_limit.data.z = (float)(*++itE);


        pid_px.data.x = (float)(*itX); pid_px.data.y = (float)(*++itX); pid_px.data.z = (float)(*++itX);
        pid_vx.data.x = (float)(*++itX); pid_vx.data.y = (float)(*++itX); pid_vx.data.z = (float)(*++itX);

        pid_py.data.x = (float)(*itY); pid_py.data.y = (float)(*++itY); pid_py.data.z = (float)(*++itY);
        pid_vy.data.x = (float)(*++itY); pid_vy.data.y = (float)(*++itY); pid_vy.data.z = (float)(*++itY);

        pid_z.data.x = (float)(*itZ); pid_z.data.y = (float)(*++itZ); pid_z.data.z = (float)(*++itZ);
        }
        catch (...) {
            return -1;
        }

}

void controller_init()
{
    pid_px.type = PID_X_POS;    pid_vx.type = PID_X_VEL;
    pid_py.type = PID_Y_POS;    pid_vy.type = PID_Y_VEL;
    pid_z.type = PID_Z;

    dest.type = DEST;
    vel_limit.type = VEL_LIM;
    err_limit.type = ERR_LIM;

    waitFlag = 0; interrupted = 0;

    car_pose_detected = 0;
    circle_pose_detected = 0;
    tape_pose_detected = 0;

    loadFromFile();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "controllerNode");

    ros::NodeHandle nh;

    position_pub = new ros::Publisher();
    *position_pub = nh.advertise<dji_sdk_controller::flight_msg>("/droneController",10);

    ros::Subscriber droneStatus_sub = nh.subscribe("droneStatus", 1, drone_status_callback);

    ros::Subscriber circle_sub = nh.subscribe("/circle_pose", 1, circle_detect_callback);
    ros::Subscriber car_sub = nh.subscribe("/car_pose", 1, car_detect_callback);
    ros::Subscriber tape_sub = nh.subscribe("/tape_pose", 100, tape_detect_callback);

    ros::AsyncSpinner spinner(4);
    spinner.start();

    controller_init();
    disp_helper();
    signal(SIGINT, catch_signal);

    waitUntilReady();
    system("clear");    disp_helper();
    command_process(0, std::cin, "INIT_LAYER");

    return 0;
}



