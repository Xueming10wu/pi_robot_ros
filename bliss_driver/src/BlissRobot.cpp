#include "BlissRobot.h"

//构造函数
BlissRobot::BlissRobot()
{
}

//构造函数
BlissRobot::BlissRobot(string serverIP_, int serverPort_) : serverIP(serverIP_), serverPort(serverPort_)
{
    startConstruction();
}

BlissRobot::~BlissRobot()
{
}

//设置IP
void BlissRobot::setServerIP(string s)
{
    serverIP = s;
}

//设置端口号
void BlissRobot::setServerPort(int port)
{
    serverPort = port;
}

//开始连接
void BlissRobot::startConstruction()
{
#ifndef USE_ROS
    signal(SIGINT, MyFunctions::stop);
#endif
    std::cout << "客户端启动，尝试连接服务端  " << serverIP << ":" << serverPort << std::endl;
    // socket
    client_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (client_fd == -1)
    {
        std::cout << "Error: socket" << std::endl;
        exit(0);
    }

    //服务端 ip及程序端口号
    serverAddr.sin_family = AF_INET; //tcp IPv4
    serverAddr.sin_port = htons(serverPort);
    serverAddr.sin_addr.s_addr = inet_addr(serverIP.c_str());

    //尝试连接服务器
    isConnected = false;
    if (connect(client_fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
    {
        std::cout << "Error: connect" << std::endl;
        exit(0);
    }
    isConnected = true;

    //序号从1开始，绝对不可以从0开始，因为会让下位机误判认为是已经接受过的数据，那么下位机只会反馈数据，不会执行这个包的命令
    Sequence = 1;
    cout << "连接成功\n";
}

//启动服务器，并接收数据
void BlissRobot::listening()
{
    //cout << "listening  "<< MyFunctions::ok() <<"\n";
#ifdef USE_ROS
    while (ros::ok())
#else
    while (MyFunctions::ok())
#endif
    {
        //收数据
        recv_len = recv(client_fd, recvBuffer, sizeof(recvBuffer), 0);
#ifdef RECV_DISPLAY
        std::cout << "共接收: " << std::dec << recv_len << "b" << std::endl;
#endif
        //检测服务端是否断开连接
        if (strcmp((char *)recvBuffer, "exit") == 0)
        {
            std::cout << "服务器断开连接" << std::endl;
            break;
        }

        switch (recv_len)
        {
        case 2:
            //接收下位机确认信息
            std::cout << "共接收: " << std::dec << recv_len << "b" << std::endl;

            if (!sendSuccess)
            { //如果已经成功，则没有再次进行校验的必要了，而且序列号等数据必定改动
                if (recvBuffer[0] == Sequence && recvBuffer[1] == send_check)
                {
                    sendSuccess = true;
                }
                else
                {
                    if (recvBuffer[0] != Sequence)
                    {
                        cout << "序列号不匹配" << endl;
                        cout << "recv[0] " << (int)recvBuffer[0] << "  ";
                        cout << "Sequence " << (int)Sequence << endl;
                    }
                    if (recvBuffer[1] != send_check)
                    {
                        cout << "check不匹配" << endl;
                    }
                }
            }

            cout << "校验结果 ";
            if (sendSuccess)
            {
                cout << "成功" << endl;
            }
            else
            {
                cout << "失败" << endl;
            }
            break;

        case LocationTCPLength:
            //接收下位机当前的信息
            if (CRC_Recv())
            { //校验位无误
                memcpy(&location, recvBuffer, LocationTCPDataLength);

                // for (int i = 0; i < recv_len; i++)
                // {
                //     std::cout << (int)recvBuffer[i] << " " ;
                // }
                // std::cout << "\n输出完毕\n\n";

                //在ros中，机械臂状态是无用的
#ifdef RECV_DISPLAY
                std::cout << "机械臂状态为 "; // << (int)location.state << std::endl;

                for (int i = 0; i < 7; i++)
                {
                    // std::cout<< "第" << i << "原始数据 ";
                    // for (int j = 0; j < 4; j ++)
                    // {
                    //     std::cout << (int)recvBuffer[ i*4 + j] << " ";
                    // }
                    //std::cout << (int)location.position[i] << std::endl;
                    std::cout << (int)location.position[i] << " ";
                }
                std::cout << std::endl
                          << std::endl
                          << std::endl
                          << std::endl;
#endif
            }
            else
            {
                std::cout << "下位机发送的数据有误，丢弃" << std::endl;
            }
            break;

        case USART_TO_TCP_SIZE:
            //接收到串口数据
            usart_recv();
            break;

        default:
            //std::cout << "接收到了"<< recv_len << "字节" << std::endl;
            break;
        }

        usleep(1000);
    }
    cout << "主动断开连接" << endl;
    close(client_fd);
}

//传输轨迹数据
void BlissRobot::sendTrajectory()
{
    //发送长度数据
    cout << "send New   NumberOfPoints:" << NumberOfPoints << endl;
    send_len = 5;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = NEW;
    sendBuffer[2] = (NumberOfPoints >> 8) & 0xff;
    sendBuffer[3] = NumberOfPoints & 0xff;
    sendModul(); //调用发送模块发送数据
    cout << "send New over\n";

    //发送轨迹数据，一个数据点的大小为116byte，一个数据包最大1395(TCP最大1460)，最多一次发送12个数据点
    NumberOfFullPackages = NumberOfPoints / FullPointInTCP;
    NumberOfRestPoints = NumberOfPoints % FullPointInTCP;

    cout << "满包数量:" << (int)NumberOfFullPackages << ", 余包包含的点数:" << (int)NumberOfRestPoints << endl;
    writePointIndex = 0;
    sendBuffer[1] = PENDING;
    if (NumberOfFullPackages > 0)
    {
        cout << "发送满包轨迹数据" << endl;
        send_len = FullTCPLength; //1395
        for (int i = 0; i < NumberOfFullPackages; i++)
        {
            sendBuffer[0] = Sequence;

            memcpy(sendBuffer + 2, &trajectory[writePointIndex], FullTCPDataLength);

            //memcpy(&fake_trajectory[writePointIndex], sendBuffer + 2, FullTCPDataLength);

            writePointIndex += FullPointInTCP;

            cout << endl;
            sendModul(); //调用发送模块发送数据
        }
    }

    //余下的轨迹数据
    if (NumberOfRestPoints > 0)
    {
        cout << "发送余包轨迹数据" << endl;
        send_len = NumberOfRestPoints * PointSize + 3;
        sendBuffer[0] = Sequence;
        memcpy(sendBuffer + 2, &trajectory[writePointIndex], send_len - 3);
        writePointIndex += NumberOfRestPoints;
        sendModul(); //调用发送模块发送数据
    }

    //发送启动标志位
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = RUNING;
    cout << "RUNING\n";
    sendModul(); //调用发送模块发送数据
}

//对接收数据进行剩余校验，校验结果，0为有错误，1为没有错误
uint8_t BlissRobot::CRC_Recv()
{
    recv_check = 0;
    for (int i = 0; i < recv_len - 1; i++)
    {
        recv_check += recvBuffer[i];
    }
    recv_check &= 0xff;

    if (recvBuffer[recv_len - 1] != recv_check)
    {
        return 0;
    }
    return 1;
}

//对发送数据进行剩余校验
void BlissRobot::CRC_Send()
{
    send_check = 0;
    for (int i = 0; i < send_len - 1; i++)
    {
        send_check += sendBuffer[i];
    }
    send_check &= 0xff;
    sendBuffer[send_len - 1] = send_check;
}

//确保发送成功的模块
void BlissRobot::sendModul()
{
    CRC_Send();
    sendSuccess = false;
    int count = 0;

#ifdef USE_ROS
    while (!sendSuccess && ros::ok())
#else
    while (!sendSuccess && MyFunctions::ok())
#endif
    {
        send(client_fd, sendBuffer, send_len, 0);
        cout << "第" << count++ << "次发送数据\n";

        //最多等待1s，否则重传
        for (int i = 0; i < 1000; i++)
        {
            if (sendSuccess)
            {
                break;
            }
            usleep(1000);
        }
    }
    Sequence++;
    Sequence %= 0xff; //0~254之间
}

//判断是否已经停止运动
bool BlissRobot::isStopped()
{
    if (location.state == STOPPED)
    {
        return true;
    }
    return false;
}

//判断是否已经到达最终位姿
bool BlissRobot::isArrived()
{
    //如果处于停止
    for (int i = 0; i < 6; i++)
    {
        if (abs(location.position[i] - trajectory[NumberOfPoints - 1].position[i]))
        {
            return false;
        }
    }
    return true;
}

//打印轨迹信息
void BlissRobot::printTrajectory()
{
    printf("轨迹点数量:%d\n", NumberOfPoints);
    for (int i = 0; i < NumberOfPoints; i++)
    {
        // volatile int32_t duration;              //运行时间
        // volatile int16_t numberOfFullPeriod[8]; //0xffff周期的数量
        // volatile int16_t restPeriod[8];         //最后余下的周期，单位为1us，只有period >= 0xffff时，才有效
        // volatile int16_t numberOfPeriod[8];     //pwm周期数
        // volatile int32_t period[8];             //产生pwm的周期，单位为1us
        // volatile int32_t position[8];           //关节运行到的脉冲位置

        printf("第%d个点: duration:%d\n"
               "numberOfFullPeriod:%d,%d,%d,%d,%d,%d,%d,%d\n"
               "restPeriod:%d,%d,%d,%d,%d,%d,%d,%d\n"
               "numberOfPeriod:%d,%d,%d,%d,%d,%d,%d,%d\n"
               "period:%d,%d,%d,%d,%d,%d,%d,%d\n"
               "position:%d,%d,%d,%d,%d,%d,%d,%d\n\n",

               i,
               trajectory[i].duration,
               trajectory[i].numberOfFullPeriod[0],
               trajectory[i].numberOfFullPeriod[1],
               trajectory[i].numberOfFullPeriod[2],
               trajectory[i].numberOfFullPeriod[3],
               trajectory[i].numberOfFullPeriod[4],
               trajectory[i].numberOfFullPeriod[5],
               trajectory[i].numberOfFullPeriod[6],
               trajectory[i].numberOfFullPeriod[7],

               trajectory[i].restPeriod[0],
               trajectory[i].restPeriod[1],
               trajectory[i].restPeriod[2],
               trajectory[i].restPeriod[3],
               trajectory[i].restPeriod[4],
               trajectory[i].restPeriod[5],
               trajectory[i].restPeriod[6],
               trajectory[i].restPeriod[7],

               trajectory[i].numberOfPeriod[0],
               trajectory[i].numberOfPeriod[1],
               trajectory[i].numberOfPeriod[2],
               trajectory[i].numberOfPeriod[3],
               trajectory[i].numberOfPeriod[4],
               trajectory[i].numberOfPeriod[5],
               trajectory[i].numberOfPeriod[6],
               trajectory[i].numberOfPeriod[7],

               trajectory[i].period[0],
               trajectory[i].period[1],
               trajectory[i].period[2],
               trajectory[i].period[3],
               trajectory[i].period[4],
               trajectory[i].period[5],
               trajectory[i].period[6],
               trajectory[i].period[7],

               trajectory[i].position[0],
               trajectory[i].position[1],
               trajectory[i].position[2],
               trajectory[i].position[3],
               trajectory[i].position[4],
               trajectory[i].position[5],
               trajectory[i].position[6],
               trajectory[i].position[7]);
    }
}

//紧急制动
void BlissRobot::robot_stop()
{
    //发送启动标志位
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = STOPPED;
    cout << "STOPPED\n";
    sendModul(); //调用发送模块发送数据
}

//重新开启Location数据的上传，默认开启
void BlissRobot::upload_start()
{
    //发送启动标志位
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = UPLOAD_START;
    cout << "UPLOAD_START\n";
    sendModul(); //调用发送模块发送数据
}

//关闭Location数据的上传，默认开启
void BlissRobot::upload_stop()
{
    //发送关闭标志位
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = UPLOAD_STOP;
    cout << "UPLOAD_STOP\n";
    sendModul(); //调用发送模块发送数据
}

//打开PWM
void BlissRobot::pwm_start()
{
    //发送pwm
    send_len = 3 + sizeof(pwm_handle);
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PWM_START;
    memcpy(sendBuffer + 2, &pwm_handle, sizeof(pwm_handle));
    cout << "PWM_START\n";
    sendModul(); //调用发送模块发送数据
}

//关闭PWM
void BlissRobot::pwm_stop()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PWM_STOP;
    cout << "PWM_STOP\n";
    sendModul(); //调用发送模块发送数据
}

//开启USART通信中断，并通过中断方式向上位机发送接收到的串口数据，需要包含波特率等信息，设置在usart_setting结构实例中
void BlissRobot::usart_start()
{
    // uint32_t BaudRate;
    // uint32_t WordLength;
    // uint32_t StopBits;
    // uint32_t Parity;
    // uint32_t Mode; //0:收发，1只收不发，2只发不收，其他数值 则默认为收发
    // uint32_t HwFlowCtl;
    // uint32_t OverSampling;
    //通常在此处配置即可
    uart_setting.BaudRate = 9600;
    uart_setting.WordLength = 0;
    uart_setting.StopBits = 0;
    uart_setting.Parity = 0;
    uart_setting.Mode = 0;
    uart_setting.HwFlowCtl = 0;
    uart_setting.OverSampling = 0;

    send_len = 3 + sizeof(uart_setting);
    sendBuffer[0] = Sequence;
    sendBuffer[1] = USART_START;
    memcpy(sendBuffer + 2, &uart_setting, sizeof(uart_setting));
    cout << "USART_START\n";
    sendModul();
}

//关闭USART通信中断
void BlissRobot::usart_stop()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = USART_STOP;
    cout << "USART_STOP\n";
    sendModul();
}

//发送数据到串口
void BlissRobot::usart_send()
{
    //获得长度   串口数据 + 序号 + 功能 + 校验
    send_len = 3 + usart_tx_len;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = USART_SEND;
    //向下发送的数据可以不定长度，仅仅放入串口数据即可
    memcpy(sendBuffer + 2, usartTXBuffer, usart_tx_len);
    cout << "USART_SEND\n";
    sendModul();
}

//接收串口数据并上传至上位机
void BlissRobot::usart_recv()
{
    /**
     * 长度信息       数据       CRC
     *    0         1-256      257
     * 
     * 共258字节长度，数据段数据最多有255位有效，其余为0
    **/

    //从结构中获取
    usart_rx_len = recvBuffer[0];

    //把数据保存在串口缓存中
    memcpy(usartRXBuffer, recvBuffer + 1, usart_rx_len);

    //打印
    cout << "接收到串口数据 长度为 : " << usart_rx_len << endl;
    for (int i = 0; i < usart_rx_len; i++)
    {
        cout << (char)usartRXBuffer[i];
    }
    cout << endl;
}

//PIN0高电平
void BlissRobot::pin0_on()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN0_ON;
    cout << "PIN0_ON\n";
    sendModul();
}

//PIN0低电平
void BlissRobot::pin0_off()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN0_OFF;
    cout << "PIN0_OFF\n";
    sendModul();
}

//PIN1高电平
void BlissRobot::pin1_on()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN1_ON;
    cout << "PIN1_ON\n";
    sendModul();
}

//PIN1低电平
void BlissRobot::pin1_off()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN1_OFF;
    cout << "PIN1_OFF\n";
    sendModul();
}

//ENABLE使能翻转
void BlissRobot::toggle_enable_pins()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = TOGGLE_ENABLE_PINS;
    cout << "TOGGLE_ENABLE_PINS\n";
    sendModul();
}

//编码器归零
void BlissRobot::encoder_reset()
{
}

//设置可动关节数量
void BlissRobot::joints_count()
{
    send_len = 4;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = JOINTS_COUNT;
    sendBuffer[2] = 6; //6个关节
    cout << "JOINTS_COUNT\n";
    sendModul();
}
