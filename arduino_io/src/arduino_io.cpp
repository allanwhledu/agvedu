#include <cstdio>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <string>

#define USHORT unsigned short int
#define BYTE unsigned char
#define CHAR char
#define UCHAR unsigned char
#define UINT unsigned int
#define DWORD unsigned int
#define PVOID void *
#define ULONG unsigned int
#define INT int
#define UINT32 UINT
#define LPVOID void *
#define BOOL BYTE
#define TRUE 1
#define FALSE 0

serial::Serial ser;
using namespace std;

int agv_state = 0;

class state_listener {
public:
  state_listener(ros::NodeHandle &nh);

  ~state_listener() {};
private:
  void stateCallback(const std_msgs::Int32 &msg);

  ros::Subscriber stateSub;
};

state_listener::state_listener(ros::NodeHandle &nh) {
  stateSub = nh.subscribe("agv_state", 1, &state_listener::stateCallback, this);
}

void state_listener::stateCallback(const std_msgs::Int32 &msg) {
  if (msg.data != agv_state) {
    agv_state = msg.data;
  }
}


int main(int argc, char **argv) {
  try {
    ser.setPort("/dev/ttyACM0");
    ser.setBaudrate(9600);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException &e) {
    ROS_ERROR_STREAM("Unable to open port ");
    return -1;
  }
  if (ser.isOpen()) {
    ROS_INFO_STREAM("Serial Port initialized");
  } else {
    return -1;
  }

  ros::init(argc, argv, "arduino_io_server");

  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  ros::Publisher goal_idx_pub =
      n.advertise<std_msgs::Int32>("/goal_idx", 1000);
  std_msgs::Int32 goal_idx_msg;

  state_listener sl(n);

  int index = 0;
  int state = 0;
  bool serialflag = true;
  std::string datastr = "";
  datastr.clear();
  while (ros::ok()) {
    if (!ser.available()) {
      ROS_INFO("serial %s", ser.available() ? "available" : "not available");
    }
    if (serialflag) {
      datastr += ser.read(ser.available());
      // ROS_INFO("%s", datastr.data());

      int success_num_read = sscanf(datastr.data(), "%*s\nSTATEDTU%dGOAL%dOVER", &state, &index);
// if(success_num_read==2)
// {
      datastr.clear();
// }

// ROS_INFO("success_num_read %d", success_num_read);
// ROS_INFO("state dtu %d", index);

      goal_idx_msg.data = index;
      goal_idx_pub.publish(goal_idx_msg);
    }
    serialflag = !serialflag;
    string stringSend;
    char charSend[50];
    sprintf(charSend, "STATEUTD%dSTATEUTD\n\r", (int) agv_state);

//    ROS_INFO("%s", charSend);
    stringSend = charSend;
    ser.write(stringSend);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
