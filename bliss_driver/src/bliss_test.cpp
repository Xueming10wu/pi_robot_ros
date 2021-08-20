#include "BlissRobot.h"





uint8_t sendBuffer[2048];               //发送缓存区
struct  Point trajectory[1024];      //路径点

int main(int argc, char *argv[])
{
    BlissRobot *blissRobotPtr = new BlissRobot("192.168.31.215", 8080);

    trajectory[0]._1 = 0;

    trajectory[0].duration[0] = 0x7f;
    trajectory[0].duration[1] = 0x6f;
    trajectory[0].duration[2] = 0xff00;
    trajectory[0].duration[3] = 0xf0f0;
    trajectory[0].duration[4] = 0xfefe;
    trajectory[0].duration[5] = 0x1564;
    trajectory[0].duration[6] = 0;


    trajectory[0].position[0] = 0xff;
    trajectory[0].position[1] = 0xff;
    trajectory[0].position[2] = 0xff;
    trajectory[0].position[3] = 0xff;
    trajectory[0].position[4] = 0xff;
    trajectory[0].position[5] = 0xffff;
    trajectory[0].position[6] = 0xff22;

    //memcpy(sendBuffer + 2, &trajectory[0], PointSize);
    // cout << "SIZE:" << sizeof(trajectory[0]) << endl;
    // cout << "输出\n";
    // for (int i = 0; i < PointSize; i += 4)
    // {
    //     cout << hex << (int)sendBuffer[i + 2] << " " 
    //             << (int)sendBuffer[i + 3] << " " 
    //             << (int)sendBuffer[i + 4] << " " 
    //             << (int)sendBuffer[i + 5] << endl;
    // }


    memcpy(sendBuffer, &trajectory[0], PointSize);
    cout << "SIZE:" << sizeof(trajectory[0]) << endl;
    cout << "输出\n";
    for (int i = 0; i < PointSize; i += 4)
    {
        cout << hex << (int)sendBuffer[i] << " " 
                << (int)sendBuffer[i + 1] << " " 
                << (int)sendBuffer[i + 2] << " " 
                << (int)sendBuffer[i + 3] << endl;
    }
    blissRobotPtr->sendBufferTest(sendBuffer, PointSize);

    return 0;
}
