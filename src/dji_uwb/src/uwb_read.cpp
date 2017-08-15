
#include <ros/ros.h>
#include <dji_uwb/usb_def.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <sys/types.h>
#include <sys/stat.h>
#include <memory>
#include <fcntl.h>      /*file control lib*/
#include <unistd.h>     /*Unix standard func*/

static const std::string dev_name = "/dev/djiUWB";
static const int data_len = sizeof(dji_uwb::UWBInfo);
static int fd;
static dji_uwb::UWBInfo data;
static ros::Publisher *uwb_pub;
static dji_uwb::uwb_msg msg;

bool uwb_read_setup()
{
  fd = open (dev.c_str(), O_RDWR);//open serial device
  if(fd==-1)
  {
    //perror("error!");
    return false;
  }

  struct termios oldtio;
  tcgetattr(fd,&oldtio);
  if(tcgetattr(fd,&oldtio)!=0)
  {
    //perror("error!");
    return false;
  }

  struct termios newtio;
  newtio.c_cflag=CIBAUD|CRTSCTS|CS8|CLOCAL|CREAD;
  cfsetispeed(&newtio, B115200);
  cfsetospeed(&newtio, B115200);
  newtio.c_iflag=IGNPAR;
  newtio.c_oflag=0;
  newtio.c_lflag=0;
  newtio.c_cc[VMIN]=1;
  newtio.c_cc[VTIME]=0;
  tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &newtio);

  return true;
}

void uwb_read_data()
{
    uint8_t buff[data_len];
    memset(buff, 0, data_len);

    msg.flag = false;
    read(fd,buff,sizeof(data_t));
    memcpy(&data, buff, sizeof(data_t));

    if(data.reserved == 0)//heand and tail check
    {
      msg.flag = true;
      msg.data.x = data.x;
      msg.data.y = data.y;
    }
}

void timerCallback(const ros::TimerEvent& e)
{
    uwb_pub.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "dji_uwb_node");
    ros::NodeHandle n;
    *uwb_pub = n.advertise<dji_uwb::uwb_msg>("/droneUWB", 1000);
    ros::Rate loop_rate(50);//50Hz for UWB receiving rate

    uwb_read_setup();
    ros::Timer timer = n.createTimer(ros::Duration(1/10), timerCallback);

    while(ros::ok())
    {
        uwb_read_data();

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
