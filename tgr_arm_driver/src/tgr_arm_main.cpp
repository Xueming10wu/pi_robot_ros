#include "TgrArmRobotRos.h"


int main(int argc, char *argv[])
{
    //节点命名
    ros::init(argc, argv, "tgr_arm_node");

    //私有参数获取句柄
    ros::NodeHandle nh("~");

    //IP和端口
    string tgr_arm_ip;
    int tgr_arm_port;

    //参数获取
    nh.param<string>("tgr_arm_ip", tgr_arm_ip, "127.0.0.1");
    nh.param<int>("tgr_arm_port", tgr_arm_port, 8080);

    //准备连接
    tgrArmRobotPtr->setServerIP(tgr_arm_ip);
    tgrArmRobotPtr->setServerPort(tgr_arm_port);


    //ros机械臂实例
    TgrArmRobotRos * tgrArmRobotRos = new TgrArmRobotRos();

    return 0;
}