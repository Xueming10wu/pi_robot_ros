#include "TgrArmRobot.h"

//构造函数
TgrArmRobot::TgrArmRobot()
{
}

//构造函数
TgrArmRobot::TgrArmRobot(string serverIP_, int serverPort_) : serverIP(serverIP_), serverPort(serverPort_)
{
    startConstruction();
}

TgrArmRobot::~TgrArmRobot()
{
}

//17位，范围为0~0x1ffff
extern int yodaEncoderRange;

void TgrArmRobot::InitJointParam()
{
    //重要参数
    //旋转+180°(+3.1415926)，需要的节拍
    plu2angel[0] = -21600; //43200;
    plu2angel[1] = -42200; //84400;        //115200;
    plu2angel[2] = -28112; //56225;       //76800;
    plu2angel[3] = -60800;
    plu2angel[4] = -44000; //51000
    plu2angel[5] = -60800; //未定

    //零点参数
    zeroPlu[0] = 0;
    zeroPlu[1] = 0;
    zeroPlu[2] = 0;
    zeroPlu[3] = 0;
    zeroPlu[4] = 0;
    zeroPlu[5] = 0;

    //固定变量,用于机械臂回归零点
    encoderHalfRange = yodaEncoderRange / 2;

    //32绿色  31红色
}

//设置IP
void TgrArmRobot::setServerIP(string s)
{
    serverIP = s;
}

//设置端口号
void TgrArmRobot::setServerPort(int port)
{
    serverPort = port;
}

//开始连接
void TgrArmRobot::startConstruction()
{
    InitJointParam();

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

    //编码器数据清零
    memset(encoderAngle, 0, sizeof(encoderAngle));

    //rs485标志位，默认为关闭
    rs485_flag = false;

    //usart标志位，默认为关闭
    usart_flag = false;

    //零点时，编码器的数据
    //因为同一型号的机械臂，在安装时也有不可避免的误差，所以每个参数都需要参考出厂标签
    encoderAngleZero[0] = 0x1a24c;
    encoderAngleZero[1] = 0xb8c8;
    encoderAngleZero[2] = 0xc71c;
    encoderAngleZero[3] = 0x1bdc3;
    encoderAngleZero[4] = 0x1c8cb;
    encoderAngleZero[5] = 0x1cc7c;

    //编码器安装方向
    encoderPositiveOrNegative[0] = 1;
    encoderPositiveOrNegative[1] = 1;
    encoderPositiveOrNegative[2] = 1;
    encoderPositiveOrNegative[3] = 1;
    encoderPositiveOrNegative[4] = -1;
    encoderPositiveOrNegative[5] = 1;

    //107084 47304 50972 114115 114863 117884


}

//断开网络
void TgrArmRobot::closeClient()
{
    cout << "断开连接" << endl;
    close(client_fd);
}

//启动服务器，并接收数据
void TgrArmRobot::listening()
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
            //std::cout << "共接收: " << std::dec << recv_len << "b" << std::endl;

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
                        std::cout << "共接收: " << std::dec << recv_len << "b" << std::endl;
                        cout << "check不匹配" << endl;
                    }
                }
            }

            if (sendSuccess)
            {
#ifdef SUCCESS_INFO
                cout << "校验结果 成功" << endl;
#endif
            }
            else
            {
                cout << "校验结果 失败" << endl;
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

            //如果为编码器的数据长度的话
            if (usart_rx_len == 11)
            {
                //校验
                if (checkEncoderData(usartRXBuffer))
                {
                    int _index = getEncoderID(usartRXBuffer) - 1;

                    //获取编码器数据
                    encoderAngle[_index] = getEncoderAngle(usartRXBuffer);
                    //encoderAngle[getEncoderID(usartRXBuffer)] = getEncoderAngle(usartRXBuffer);
#ifdef SUCCESS_INFO
                    std::cout << "编码器" << _index + 1 << "数值为" << encoderAngle[_index] << std::endl;
#endif
                }
                else
                {
                    std::cout << "编码器数据校验失败" << std::endl;
                }
            }

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

//心跳检测
void TgrArmRobot::keepAlive()
{
}

//传输轨迹数据
void TgrArmRobot::sendTrajectory()
{
    //发送长度数据
#ifdef SUCCESS_INFO
    cout << "send New   NumberOfPoints:" << NumberOfPoints << endl;
#endif
    send_len = 5;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = NEW;
    sendBuffer[2] = (NumberOfPoints >> 8) & 0xff;
    sendBuffer[3] = NumberOfPoints & 0xff;
    sendModul(); //调用发送模块发送数据
#ifdef SUCCESS_INFO
    cout << "send New over\n";
#endif

    //发送轨迹数据，一个数据点的大小为116byte，一个数据包最大1395(TCP最大1460)，最多一次发送12个数据点
    NumberOfFullPackages = NumberOfPoints / FullPointInTCP;
    NumberOfRestPoints = NumberOfPoints % FullPointInTCP;
#ifdef SUCCESS_INFO
    cout << "满包数量:" << (int)NumberOfFullPackages << ", 余包包含的点数:" << (int)NumberOfRestPoints << endl;
#endif
    writePointIndex = 0;
    sendBuffer[1] = PENDING;
    if (NumberOfFullPackages > 0)
    {
#ifdef SUCCESS_INFO
        cout << "发送满包轨迹数据" << endl;
#endif
        send_len = FullTCPLength; //1395
        for (int i = 0; i < NumberOfFullPackages; i++)
        {
            sendBuffer[0] = Sequence;

            memcpy(sendBuffer + 2, &trajectory[writePointIndex], FullTCPDataLength);

            //memcpy(&fake_trajectory[writePointIndex], sendBuffer + 2, FullTCPDataLength);

            writePointIndex += FullPointInTCP;

            sendModul(); //调用发送模块发送数据
        }
    }

    //余下的轨迹数据
    if (NumberOfRestPoints > 0)
    {
#ifdef SUCCESS_INFO
        cout << "发送余包轨迹数据" << endl;
#endif
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
#ifdef SUCCESS_INFO
    cout << "RUNING\n";
#endif
    sendModul(); //调用发送模块发送数据
}

//对接收数据进行剩余校验，校验结果，0为有错误，1为没有错误
uint8_t TgrArmRobot::CRC_Recv()
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
void TgrArmRobot::CRC_Send()
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
void TgrArmRobot::sendModul()
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

        if (count != 0)
        {
            cout << "第" << count++ << "次发送数据\n";
        }

        //最多等待1s，否则重传
        for (int i = 0; i < 1000; i++)
        {
            if (sendSuccess)
            {
                break;
            }
            usleep(1000);
        }

        //最多重传5次，否则视为断开链接
        if (count > 5)
        {
            closeClient();
            ros::shutdown();
            exit(0);
        }
    }
    Sequence++;
    Sequence %= 0xff; //0~254之间
}

//判断是否已经停止运动
bool TgrArmRobot::isStopped()
{
    if (location.state == STOPPED)
    {
        return true;
    }
    return false;
}

//判断是否已经到达最终位姿
bool TgrArmRobot::isArrived()
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
void TgrArmRobot::printTrajectory()
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
void TgrArmRobot::robot_stop()
{
    //发送启动标志位
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = STOPPED;
    cout << "STOPPED\n";
    sendModul(); //调用发送模块发送数据
}

//重新开启Location数据的上传，默认开启
void TgrArmRobot::upload_start()
{
    //发送启动标志位
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = UPLOAD_START;
#ifdef SUCCESS_INFO
    cout << "UPLOAD_START\n";
#endif
    sendModul(); //调用发送模块发送数据
}

//关闭Location数据的上传，默认开启
void TgrArmRobot::upload_stop()
{
    //发送关闭标志位
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = UPLOAD_STOP;
#ifdef SUCCESS_INFO
    cout << "UPLOAD_STOP\n";
#endif
    sendModul(); //调用发送模块发送数据
}

//打开PWM
void TgrArmRobot::pwm_start()
{
    //发送pwm
    send_len = 3 + sizeof(pwm_handle);
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PWM_START;
    memcpy(sendBuffer + 2, &pwm_handle, sizeof(pwm_handle));
#ifdef SUCCESS_INFO
    cout << "PWM_START\n";
#endif
    sendModul(); //调用发送模块发送数据
}

//关闭PWM
void TgrArmRobot::pwm_stop()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PWM_STOP;
#ifdef SUCCESS_INFO
    cout << "PWM_STOP\n";
#endif
    sendModul(); //调用发送模块发送数据
}

//开启USART通信中断，并通过中断方式向上位机发送接收到的串口数据，需要包含波特率等信息，设置在usart_setting结构实例中
void TgrArmRobot::usart_start()
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
#ifdef SUCCESS_INFO
    cout << "USART_START\n";
#endif
    sendModul();

    usart_flag = true;
}

//关闭USART通信中断
void TgrArmRobot::usart_stop()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = USART_STOP;
#ifdef SUCCESS_INFO
    cout << "USART_STOP\n";
#endif
    sendModul();

    usart_flag = false;
}

//发送数据到串口
void TgrArmRobot::usart_send()
{
    //获得长度   串口数据 + 序号 + 功能 + 校验
    send_len = 3 + usart_tx_len;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = USART_SEND;
    //向下发送的数据可以不定长度，仅仅放入串口数据即可
    memcpy(sendBuffer + 2, usartTXBuffer, usart_tx_len);
#ifdef SUCCESS_INFO
    cout << "USART_SEND\n";
#endif
    sendModul();
}

//接收串口数据并上传至上位机
void TgrArmRobot::usart_recv()
{
    /**
     * 长度信息       数据       CRC
     *    0         1-255      256
     * 
     * 共258字节长度，数据段数据最多有255位有效，其余为0
    **/

    //从结构中获取
    usart_rx_len = recvBuffer[0];

    //把数据保存在串口缓存中
    memcpy(usartRXBuffer, recvBuffer + 1, usart_rx_len);

#ifdef SUCCESS_INFO
    //打印
    cout << "接收到串口数据 长度为 : " << usart_rx_len << endl;
    for (int i = 0; i < usart_rx_len; i++)
    {
        cout << (char)usartRXBuffer[i];
    }
    cout << hex << endl;
    for (int i = 0; i < usart_rx_len; i++)
    {
        cout << (int)usartRXBuffer[i] << " ";
    }
    cout << dec << endl;
#endif
}

//PIN0高电平
void TgrArmRobot::pin0_on()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN0_ON;
    cout << "PIN0_ON\n";
    sendModul();
}

//PIN0低电平
void TgrArmRobot::pin0_off()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN0_OFF;
    cout << "PIN0_OFF\n";
    sendModul();
}

//PIN1高电平
void TgrArmRobot::pin1_on()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN1_ON;
    cout << "PIN1_ON\n";
    sendModul();
}

//PIN1低电平
void TgrArmRobot::pin1_off()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = PIN1_OFF;
    cout << "PIN1_OFF\n";
    sendModul();
}

//ENABLE使能翻转
void TgrArmRobot::toggle_enable_pins()
{
    send_len = 5;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = TOGGLE_ENABLE_PINS;
    sendBuffer[2] = (enable_pins >> 8) & 0xff;
    sendBuffer[3] = enable_pins & 0xff;
    cout << "TOGGLE_ENABLE_PINS\n";
    sendModul();
}

//RS485使能
void TgrArmRobot::rs485_enable()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = RS485_ENABLE;
#ifdef SUCCESS_INFO
    cout << "RS485_ENABLE\n";
#endif
    sendModul();

    rs485_flag = true;
}

//RS485失能
void TgrArmRobot::rs485_disable()
{
    send_len = 3;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = RS485_DISABLE;
#ifdef SUCCESS_INFO
    cout << "RS485_DISABLE\n";
#endif
    sendModul();

    rs485_flag = false;
}

//设置可动关节数量
void TgrArmRobot::joints_count()
{
    send_len = 4;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = JOINTS_COUNT;
    sendBuffer[2] = 6; //6个关节
    cout << "JOINTS_COUNT\n";
    sendModul();
}

//手动设置LOCATION，给当前机械臂位置赋值，用于机械臂位置校准,通过location_setting实例进行设置，然后再调用此函数
//但是必须在ros机械臂完全停止运动的时候进行，同时建议在调用此函数前进行
void TgrArmRobot::location_setting()
{
    send_len = 3 + LocationTCPDataLength;
    sendBuffer[0] = Sequence;
    sendBuffer[1] = LOCATION_SETTING;
    memcpy(sendBuffer + 2, &location_setting_handle, LocationTCPDataLength);
    cout << "LOCATION_SETTING\n";
    sendModul();
}

extern uint8_t yodaEncoderDataFrame[6][8]; //引入编码器数据帧

void TgrArmRobot::getEncoders()
{
    if ((!usart_flag) || (!rs485_flag))
    {
        //关闭巡回资源
        upload_stop();

        //使能485
        rs485_enable();
        usleep(100000);

        //开启串口通信
        usart_start();
        usleep(100000);
    }

    //根据编码器工厂进行通信设计，如果编码器是被动方式，数据放入到缓存中
    usart_tx_len = 8;

    for (int i = 0; i < 6; i++)
    {
        memcpy(usartTXBuffer, yodaEncoderDataFrame[i], usart_tx_len);
        usart_send();  //调用发送
        usleep(50000); //50ms
    }

    // //关闭串口通信
    usart_stop();
    usleep(100000);

    // //关闭485
    rs485_disable();
    usleep(100000);

    //恢复巡回资源
    upload_start();
    usleep(100000);

    //#ifdef SUCCESS_INFO
    //输出数据
    cout << "编码器数据为:";
    for (int i = 0; i < 6; i++)
    {
        cout << encoderAngle[i] << " ";
    }
    cout << endl;

    for (int i = 0; i < 6; i++)
    {
        cout << hex << encoderAngle[i] << " ";
    }
    cout << dec << endl
         << endl;
    //#endif
}

//通过对编码器的读取，获取机器人各个关节的角度
void TgrArmRobot::getPose()
{
    //等待获得编码器数据
    getEncoders();

    //计算脉冲数量
    for (int i = 0; i < 6; i++)
    {
        //旋转角度需要小于180度，怎么近怎么转
        temp_a = encoderAngle[i] - encoderAngleZero[i];
        if (abs(temp_a) > encoderHalfRange)
        {
            if (temp_a > encoderHalfRange)
            {
                temp_a -= yodaEncoderRange;
            }
            else if (temp_a < -encoderHalfRange)
            {
                temp_a += yodaEncoderRange;
            }
        }

        // 移动角度 = (当前编码器位置-编码器零点位置) * 2PI/编码器范围
        // 脉冲数量 = (移动角度/PI) * 脉冲参数 =
        // 2* (当前编码器位置-编码器零点位置) * 脉冲参数/编码器范围

        temp_b = temp_a * plu2angel[i] * encoderPositiveOrNegative[i];
        temp_c = temp_b / encoderHalfRange;

        cout << "\033[31m第" << i << "应当移动\033[0m"
             << temp_b << " " << temp_c << endl;

        //存放获取到的位置信息
        location_setting_handle.position[i] = temp_c;
    }
}

void TgrArmRobot::return_to_zero()
{
    cout << "return_to_zero\n";

    //设置一个执行点
    NumberOfPoints = 1;

    //先进行清零
    memset(&trajectory[0], 0, PointSize);

    //获得各个关节的脉冲角度信息，并存放在location_setting_handle中
    getPose();

    //计算脉冲数量
    for (int i = 0; i < 6; i++)
    {
        trajectory[0].position[i] =  -location_setting_handle.position[i];
    }

    //无需运动的关节，将于10ms关闭
    trajectory[0].period[0] = 1000;
    trajectory[0].period[1] = 1000;
    trajectory[0].period[2] = 1000;
    trajectory[0].period[3] = 1000;
    trajectory[0].period[4] = 1000;
    trajectory[0].period[5] = 1000;

    //设置执行速度，慢点稳点,5s恢复
    trajectory[0].duration = 5000000;

    //调用tgrArmRobot中的sendTrajectory进行发送数据的操作

    //#ifdef SUCCESS_INFO
    //printTrajectory();
    //#endif

    //发送数据
    sendTrajectory();
}
