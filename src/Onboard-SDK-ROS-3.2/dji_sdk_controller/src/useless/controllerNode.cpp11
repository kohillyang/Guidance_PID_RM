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

#include <iostream>
#include <boost/algorithm/string.hpp>
#include <ros/ros.h>
#include "ctrl_type.h"
#include <opencv2/opencv.hpp>

static std::string cmd;

dji_sdk_controller::flight_msg pid_px, pid_py, pid_z;
dji_sdk_controller::flight_msg pid_vx, pid_vy;
dji_sdk_controller::flight_msg dest, vel_limit, err_limit;


void disp_helper()
{
    std::cout << "            Remote Controller Helper             " <<std::endl;
    std::cout << " =============================================== " <<std::endl;
    std::cout << "   [TAKEOFF] Takeoff                             " <<std::endl;
    std::cout << "   [L/Landing] Landing                           " <<std::endl;
    std::cout << "   [XP] PID set for x-axis position             " <<std::endl;
    std::cout << "   [YP] PID set for x-axis position             " <<std::endl;
    std::cout << "   [PZ] PID set for x-axis position             " <<std::endl;
    std::cout << "   [XV] PID set for x-axis velocity             " <<std::endl;
    std::cout << "   [YV] PID set for x-axis velocity             " <<std::endl;
    std::cout << "   [D] Destination Coordinate Set                " <<std::endl;
    std::cout << "   [VEL] Velocity Limit Set for Translation      " <<std::endl;
    std::cout << "   [ERR] Location Limit Set for Translation      " <<std::endl;
    std::cout << "   [PB] Publish Current Params Once              " <<std::endl;
    std::cout << "   [WF/W] Write Params to File                   " <<std::endl;
    std::cout << "   [RR/R] Reset to Default                       " <<std::endl;
    std::cout << "   [RF] Reset to File                            " <<std::endl;
    std::cout << "   [P/Print] Print current parameters.           " <<std::endl;
    std::cout << "   [C/Clear/Cls] Clear Screen                    " <<std::endl;
    std::cout << "   [Q/Quit/Exit] Quit                            " <<std::endl;
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

void publishAll(const ros::Publisher &position_pub)
{
    position_pub.publish(pid_px); position_pub.publish(pid_py); position_pub.publish(pid_z);

    position_pub.publish(pid_vx); position_pub.publish(pid_vy);

    position_pub.publish(vel_limit);
    position_pub.publish(err_limit);
    position_pub.publish(dest);
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

int writeToFile(std::string file_name = "")
{
    return -1;
    /*if(file_name.empty())
    {
        file_name = "/home/ubuntu/flight_param.yml";
    }

    try {
        cv::FileStorage fs(file_name, cv::FileStorage::WRITE);

        fs<<"expect_x"<<dest.data.x; fs<<"expect_y"<<dest.data.y; fs<<"expect_z"<<dest.data.z;

        fs<<"pid_x_kp"<<pid_x.data.x; fs<<"pid_x_ki"<<pid_x.data.y; fs<<"pid_x_kd"<<pid_x.data.z;
        fs<<"pid_y_kp"<<pid_y.data.x; fs<<"pid_y_ki"<<pid_y.data.y; fs<<"pid_y_kd"<<pid_y.data.z;
        fs<<"pid_z_kp"<<pid_z.data.x; fs<<"pid_z_ki"<<pid_z.data.y; fs<<"pid_z_kd"<<pid_z.data.z;

        fs<<"x_limit"<<vel_limit.data.x; fs<<"y_limit"<<vel_limit.data.y; fs<<"z_limit"<<vel_limit.data.z;
        fs<<"x_land"<<err_limit.data.x; fs<<"y_land"<<err_limit.data.y; fs<<"z_land"<<err_limit.data.z;

        return 0;
    }
    catch (...) {
        return -1;
    }*/

}

void param_reset(const ros::Publisher &position_pub, int type)
{
    switch(type)
    {
    case RESET_TO_FILE:
        if(loadFromFile(cmd) >=0 )
        {
            publishAll(position_pub);
        }
        else{
            std::cout<< "WRONG FILE NAME INPUT!" <<std::endl;
        }
        break;
    case RESET_TO_DEFAULT:
        loadFromFile();
        publishAll(position_pub);
        break;
    default:
        break;
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

    loadFromFile();
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "controllerNode");

    ros::NodeHandle nh;
    ros::Publisher position_pub = nh.advertise<dji_sdk_controller::flight_msg>("/droneController",10);

    dji_sdk_controller::flight_msg msg;
    int cmd_right = 0;

    controller_init();
    disp_helper();

    while(1)
    {   
        std::cin >> cmd;
        boost::algorithm::to_upper(cmd);

        if (cmd.compare("L") == 0||cmd.compare("LAND") == 0||cmd.compare("LANDING") == 0)             //0x07
        {
            msg.type = LANDING;
            msg.data.x=0; msg.data.y=0; msg.data.z=0;
        }
        else if (cmd.compare("TAKEOFF") == 0)             //0x08
        {
            msg.type = TAKEOFF;
            msg.data.x=0; msg.data.y=0; msg.data.z=0;
        }
        else if (cmd.compare("XP") == 0)             //0x04
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = PID_X_POS;
            pid_px.data = msg.data;
        }
        else if(cmd.compare("YP") == 0)         //0x05
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = PID_Y_POS;
            pid_py.data = msg.data;
        }
        else if (cmd.compare("XV") == 0)             //0x04
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = PID_X_VEL;
            pid_vx.data = msg.data;
        }
        else if(cmd.compare("YV") == 0)         //0x05
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = PID_Y_VEL;
            pid_vy.data = msg.data;
        }
        else if(cmd.compare("PZ") == 0)         //0x06
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = PID_Z;
            pid_z.data = msg.data;
        }
        else if(cmd.compare("D") == 0)          //0x01
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = DEST;
            dest.data = msg.data;
        }
        else if(cmd.compare("VEL") == 0)        //0x02
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = VEL_LIM;
            vel_limit.data = msg.data;
        }
        else if(cmd.compare("ERR") == 0)        //0x03
        {
            std::cin >> msg.data.x >> msg.data.y >> msg.data.z;
            msg.type = ERR_LIM;
            err_limit.data = msg.data;
        }
        else if(cmd.compare("RR") == 0||cmd.compare("R") == 0)         //USER, Reset to Default
        {
            cmd_right = 1;
            param_reset(position_pub, RESET_TO_DEFAULT);
            disp_param();
            std::cout<<std::endl << "[Success!]Please Select Another One:             " <<std::endl;
        }
        else if(cmd.compare("RF") == 0)         //USER, Reset to <Filename>
        {
            cmd_right = 1;
            std::cin >> cmd;
            param_reset(position_pub, RESET_TO_FILE);
            disp_param();
            std::cout<<std::endl << "[Success!]Please Select Another One:             " <<std::endl;
        }
        else if(cmd.compare("WF") == 0||cmd.compare("W") == 0)         //USER, Write to <Filename>
        {
            cmd_right = 1;
            std::cin >> cmd;
            writeToFile(cmd);
            disp_param();
            std::cout<<std::endl << "[Success!]Please Select Another One:             " <<std::endl;
        }
        else if(cmd.compare("PB")==0)           //USER, publish all params
        {
            cmd_right = 1;
            system("clear");
            publishAll(position_pub);
            std::cout << "ALL Param published as follow:" << std::endl;
            disp_param();
            disp_helper();
        }
        else if(cmd.compare("P")==0||cmd.compare("PRINT")==0||cmd.compare("DISPLAY")==0)
        {
            cmd_right = 1;
            system("clear");
            disp_param();
            disp_helper();
        }
        else if(cmd.compare("C")==0||cmd.compare("CLEAR")==0||cmd.compare("CLS")==0)
        {
            cmd_right = 1;
            system("clear");
            disp_helper();
        }
        else if(cmd.compare("Q")==0||cmd.compare("EXIT")==0||cmd.compare("QUIT")==0)
        {
            system("clear");
            std::cout << "bye." << std::endl;
            exit(-1);
        }
        else{
            cmd_right = 1;
            system("clear");
            std::cout << "Undeclared Commands." << std::endl << std::endl;
            disp_helper();
        }

        if(cmd_right==0)
        {
            position_pub.publish(msg);
            std::cout<<std::endl << "[(" <<msg.type<<"), (" <<msg.data.x<<"," <<msg.data.y<<"," <<msg.data.z <<") ] published."<<std::endl;
            std::cout << "[Success!]Please Select Another One:             " <<std::endl;
        }
        else
        {
            cmd_right = 0;
        }
    }

    return 0;
}



