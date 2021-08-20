#include "BlissRobotRos.h"


int main(int argc, char *argv[])
{
    //节点命名
    ros::init(argc, argv, "bliss_node");

    //私有参数获取句柄
    ros::NodeHandle nh("~");

    //IP和端口
    string bliss_ip;
    int bliss_port;

    //参数获取
    nh.param<string>("bliss_ip", bliss_ip, "192.168.31.215");
    nh.param<int>("bliss_port", bliss_port, 8080);

    //准备连接
    blissRobotPtr->setServerIP(bliss_ip);
    blissRobotPtr->setServerPort(bliss_port);


    //ros机械臂实例
    BlissRobotRos * blissRobotRos = new BlissRobotRos();

    return 0;
}