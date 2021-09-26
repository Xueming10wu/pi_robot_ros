#ifndef TGRARMROBOT_H
#define TGRARMROBOT_H

#include "common.h"

#define PointSize 116            //每个Point轨迹点所占的字节数量   8x(3x2+2x4)+4 = 116
#define FullPointInTCP 12        //每个TCP数据包中，包含的最大字节数量
#define FullTCPDataLength 1392   //116×12 = 1392
#define FullTCPLength 1395       //1392 + 3 = 1395
#define LocationTCPDataLength 33 //4x8 + 1 = 33
#define LocationTCPLength 34     //33 + 1 = 34
#define USART_TO_TCP_SIZE 258    //串口转网口的数据固定长度 长度 + 256 + CRC

#define USART_RXBUFFER_SIZE 256
#define USART_TXBUFFER_SIZE 256

struct Point
{
    volatile int32_t duration;              //运行时间，单位为1us
    volatile int16_t numberOfFullPeriod[8]; //0xffff周期的数量
    volatile int16_t restPeriod[8];         //最后余下的周期，单位为1us，只有period >= 0xffff时，才有效
    volatile int16_t numberOfPeriod[8];     //pwm周期数
    volatile int32_t period[8];             //产生pwm的周期，单位为1us
    volatile int32_t position[8];           //关节运行到的脉冲位置
};

struct Location
{
    volatile int32_t position[8];
    volatile int8_t state;
};

//自STM32库函数中引入，除波特率外，通常赋值0即可  除Mode外，其他可直接参考STM32库函数
struct UART_InitTypeDef
{
    uint32_t BaudRate;
    uint32_t WordLength;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode; //0:收发，1只收不发，2只发不收，其他数值 则默认为收发
    uint32_t HwFlowCtl;
    uint32_t OverSampling;
};

//依赖PWM的末端抓取装置，用于舵机(servo)和步进电机(step motor)
struct PWM
{
    //PSC ARR CCR1有效位数都为16位数，即需要控制在0-0xffff之间
    int32_t PSC;        // PRC，预分频器系数
    int32_t ARR;        // ARR，自动重载寄存器
    int32_t CCR1;       // CCR1，捕获/比较寄存器1，只使用通道1
    int32_t PluseCount; // 执行脉冲的数量，只有此位不为0时有效，可为负数
};

//状态值
enum
{
    NEW = 0x00,                //新数据，获取轨迹数据的长度
    PENDING = 0x01,            //悬起态，读取轨迹数据
    RUNING = 0x02,             //执行态，执行轨迹
    STOPPED = 0x03,            //静止态，处于数据执行完毕的状态
    UPLOAD_START = 0x04,       //重新开启Location数据的上传，默认开启
    UPLOAD_STOP = 0x05,        //关闭Location数据的上传，默认开启
    PWM_START = 0x06,          //打开PWM
    PWM_STOP = 0x07,           //关闭PWM
    USART_START = 0x08,        //开启USART通信中断，并通过中断方式向上位机发送接收到的串口数据，需要包含波特率等信息
    USART_STOP = 0x09,         //关闭USART通信中断
    USART_SEND = 0x0a,         //发送数据到串口
    USART_RECV = 0x0b,         //接收串口数据并上传至上位机
    PIN0_ON = 0x0c,            //GPIO_0高电平
    PIN0_OFF = 0x0d,           //GPIO_0低电平
    PIN1_ON = 0x0e,            //GPIO_1高电平
    PIN1_OFF = 0x0f,           //GPIO_1低电平
    TOGGLE_ENABLE_PINS = 0x10, //ENABLE使能翻转，包括夹具，可以做到失力和使能的切换
    RETURN = 0x11,             //返回零点
    JOINTS_COUNT = 0x12,       //设置关节数量，默认为6轴，每次连接非6轴机械臂之后2s才能调用此命令，比较影响效率，尽量不要使用此接口，
                               //如果使用7轴、8轴，可以联系控制器售后，直接在出场时设计成默认7轴的机械臂控制器
    LOCATION_SETTING = 0x13    //手动设置location的数值，给当前机械臂位置赋值，用于机械臂位置校准,通过location对象即可
} ROBOTSTATE;

class TgrArmRobot
{
public:
    //友元函数
    friend void listen(TgrArmRobot *p);

    //构造函数
    TgrArmRobot();

    //构造函数，参数为 IP地址以及端口号
    TgrArmRobot(string serverIP, int serverPort_);

    //析构函数
    ~TgrArmRobot();

    //设置IP
    void setServerIP(string s);

    //设置端口号
    void setServerPort(int port);

    //开始连接
    void startConstruction();

    //断开网络
    void closeClient();

    //启动服务器，并接收数据
    void listening();

    //传输轨迹数据
    void sendTrajectory();

    //判断是否已经停止运动
    bool isStopped();

    //判断是否已经到达最终位姿
    bool isArrived();

    //打印轨迹信息
    void printTrajectory();

    /*
     *  辅助功能
     */

    //紧急制动
    void robot_stop();

    //重新开启Location数据的上传，默认开启
    void upload_start();

    //关闭Location数据的上传，默认开启
    void upload_stop();

    //打开PWM
    void pwm_start();

    //关闭PWM
    void pwm_stop();

    //开启USART通信中断，并通过中断方式向上位机发送接收到的串口数据，需要包含波特率等信息，设置在usart_setting结构实例中
    void usart_start();

    //关闭USART通信中断
    void usart_stop();

    //发送数据到串口
    void usart_send();

    //接收串口数据并上传至上位机
    void usart_recv();

    //PIN0高电平
    void pin0_on();

    //PIN0低电平
    void pin0_off();

    //PIN1高电平
    void pin1_on();

    //PIN1低电平
    void pin1_off();

    //ENABLE使能翻转
    void toggle_enable_pins();

    //编码器归零，即将当前位置作为编码器零点位置，需要配合串口等基础指令
    void encoder_reset();

    //设置可动关节数量
    void joints_count();

    //手动设置LOCATION，给当前机械臂位置赋值，用于机械臂位置校准,通过location_setting对象进行
    //但是必须在ros机械臂完全停止运动的时候进行
    void location_setting();

    //接口控制数据
    //struct Point fake_trajectory[1024];

    uint16_t NumberOfPoints;                 //轨迹点的数量
    struct Point trajectory[512];            //路径点，f4 512,   h7 1024
    struct Location location;                //当前机械臂末端位置和状态,用于数据反馈
    struct Location location_setting_handle; //机械臂位置设置实例
    struct UART_InitTypeDef uart_setting;    //串口设置对象
    struct PWM pwm_handle;                   //pwm实例

    volatile int usart_rx_len;  //串口接收数据长度
    volatile int usart_tx_len;  //串口发送数据长度
    uint8_t usartRXBuffer[256]; //串口接收缓存区
    uint8_t usartTXBuffer[256]; //串口发送缓存区

private:
    uint8_t CRC_Recv();
    void CRC_Send();

    void sendModul(); //确保发送成功的模块

    int client_fd;                 //tcp对象实例
    struct sockaddr_in serverAddr; //协议族

    string serverIP; //服务程序IP地址
    int serverPort;  //服务程序端口号

    bool isConnected;          //是否连接  true已连接  false未连接
    volatile bool sendSuccess; //发送成功标志位

    volatile uint16_t recv_len; //接收长度
    volatile uint16_t send_len; //发送长度
    uint16_t writePointIndex;   //当前写入点的索引

    volatile uint8_t Sequence;    //包的序列号
    uint8_t NumberOfFullPackages; //长度为 FullTCPLength 的包的数量  0～255个
    uint8_t NumberOfRestPoints;   //发完长 FullTCPLength 的包后，还余下来点的数量

    volatile uint8_t recv_check; //接收数据 校验计算结果
    volatile uint8_t send_check; //发送数据 计算出要发送的校验位

    uint8_t recvBuffer[1024]; //网络接收缓存区
    uint8_t sendBuffer[2048]; //网络发送缓存区
};

#endif //TGRARMROBOT_H