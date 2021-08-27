#include "BlissRobot.h"

using namespace std;

//BlissRobot 监听友元函数
void listening(BlissRobot *p)
{
    p->listening();
}

void sendTrajectory(BlissRobot *p)
{
    p->sendTest();
}


int main(int argc, char *argv[])
{

    BlissRobot *blissRobotPtr = new BlissRobot("192.168.31.215", 8080);
    

    //std::thread t_listening = std::thread(listening, blissRobotPtr);
    //std::thread t_sending = std::thread(sendTrajectory, blissRobotPtr);
    sleep(1);
    //blissRobotPtr->sendOnePoint();
    
    sendTrajectory(blissRobotPtr);
    //if (t_listening.joinable() && t_sending.joinable())
    // if (t_listening.joinable())
    // {
    //     t_listening.join();
    //     //t_sending.join();
    // }
    // else
    // {
    //     std::cout << "thread cannot join" << std::endl;
    // }
    cout << "退出\n";

    //ros::shutdown();

    return 0;
}