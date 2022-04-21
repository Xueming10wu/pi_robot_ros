#include "TgrArmRobotRos.h"

//机械臂实例
TgrArmRobot *tgrArmRobotPtr = new TgrArmRobot();

//TgrArmRobot 监听友元函数
void listening(TgrArmRobot *p)
{
    std::cout << "void listening \n";
    p->listening();
}

TgrArmRobotRos::TgrArmRobotRos()
{
    //句柄实例
    ros::NodeHandle nh;

    //动作名称
    action_name = "/tgr_arm/tgr_arm_controller/follow_joint_trajectory";

    //初始化关节变量
    joint_msg.name.resize(6);
    joint_msg.position.resize(6);
    joint_msg.header.frame_id = "/tgr_arm";

    //初始化ros_feedback
    ros_feedback.header.frame_id = "/tgr_arm";
    ros_feedback.desired.positions.resize(6);
    ros_feedback.actual.positions.resize(6);

    //关节命名
    joint_msg.name[0] = "joint0";
    joint_msg.name[1] = "joint1";
    joint_msg.name[2] = "joint2";
    joint_msg.name[3] = "joint3";
    joint_msg.name[4] = "joint4";
    joint_msg.name[5] = "joint5";

    //各个关节执行完毕所需的时间
    memset(durations, 0, sizeof(durations));
    duration_sum = 0;

    //功能
    extra_features_msg.Tag = 0;
    extra_features_msg.Position.resize(6);





    //启动
    tgrArmRobotPtr->startConstruction();

    //监听机械臂
    std::thread t_listening = std::thread(listening, tgrArmRobotPtr);

    //关节发布者初始化
    joint_pub = nh.advertise<sensor_msgs::JointState>("/tgr_arm/joint_states", 1);

    //服务器初始化
    as = new Server(nh, action_name, boost::bind(&TgrArmRobotRos::executeCB, this, _1), false);

    //服务器开启
    as->start();

    //功能订阅者初始化
    extra_features_sub = nh.subscribe("/tgr_arm/extraFeatures", 1, &TgrArmRobotRos::extraFeaturesCB, this);

    ros::AsyncSpinner spinner(1);
    spinner.start();


    //获取机器人各个关节的脉冲角度
    tgrArmRobotPtr->getPose();

    usleep(100000); //0.1s间隔

    //写入到控制器中
    tgrArmRobotPtr->location_setting();

    //回归零点
    //tgrArmRobotPtr->return_to_zero();


    //更新关节数据
    jointStateUpdate();

    if (t_listening.joinable())
    {
        t_listening.join();
    }
    else
    {
        std::cout << "thread cannot join" << std::endl;
    }
    std::cout << "退出\n";
    ros::shutdown();
}

TgrArmRobotRos::~TgrArmRobotRos()
{
}

//extraFeatures功能回调函数
void TgrArmRobotRos::extraFeaturesCB(const tgr_arm_driver::ExtraFeaturesConstPtr &msg)
{
    //获取消息标签
    extra_features_msg.Tag = msg->Tag;

    switch (msg->Tag)
    {
    case STOPPED:
        //紧急制动
        tgrArmRobotPtr->robot_stop();
        break;

    case UPLOAD_START:
        //重新开启Location数据的上传，默认开启
        tgrArmRobotPtr->upload_start();
        break;

    case UPLOAD_STOP:
        //关闭Location数据的上传，默认开启
        tgrArmRobotPtr->upload_stop();
        break;

    case PWM_START:
        //获取数据
        tgrArmRobotPtr->pwm_handle.PSC = msg->PSC;
        tgrArmRobotPtr->pwm_handle.ARR = msg->ARR;
        tgrArmRobotPtr->pwm_handle.CCR1 = msg->CCR1;
        tgrArmRobotPtr->pwm_handle.PluseCount = msg->PluseCount;
        //发送pwm数据
        tgrArmRobotPtr->pwm_start();
        break;

    case PWM_STOP:
        //关闭pwm
        tgrArmRobotPtr->pwm_stop();
        break;

    case PIN0_ON:
        tgrArmRobotPtr->pin0_on();
        break;

    case PIN0_OFF:
        tgrArmRobotPtr->pin0_off();
        break;

    case PIN1_ON:
        tgrArmRobotPtr->pin1_on();
        break;

    case PIN1_OFF:
        tgrArmRobotPtr->pin1_off();
        break;

    case TOGGLE_ENABLE_PINS:    //12
        /*
         * 关节序号  0   1   2   3   4   5   6   7     8pwm    全部
         * 输入数值  1   2   4   8   16  32  64  128   256     (511)0x1ff
         * 
         * 可以进行各种使能组合,不过为了方便,建议在终端操作时,采用上述几个数值
         */

        tgrArmRobotPtr->enable_pins = msg->PSC & 0x01ff;
        cout << "tgrArmRobotPtr->enable_pins " << tgrArmRobotPtr->enable_pins << endl;
        tgrArmRobotPtr->toggle_enable_pins();
        break;

    case USART_START: //13
        //开启串口通信
        tgrArmRobotPtr->usart_start();
        break;

    case USART_STOP: //14
        //关闭USART通信中断
        tgrArmRobotPtr->usart_stop();
        break;

    case RS485_ENABLE: //17
        tgrArmRobotPtr->rs485_enable();
        break;

    case RS485_DISABLE: //18
        tgrArmRobotPtr->rs485_disable();
        break;

    case LOCATION_SETTING:
        //设置当前角度脉冲数值,必须要在机械臂完全停止运动的时候使用这个功能
        tgrArmRobotPtr->location_setting_handle.state = LOCATION_SETTING;

        //6轴
        tgrArmRobotPtr->location_setting_handle.position[0] = msg->Position[0];
        tgrArmRobotPtr->location_setting_handle.position[1] = msg->Position[1];
        tgrArmRobotPtr->location_setting_handle.position[2] = msg->Position[2];
        tgrArmRobotPtr->location_setting_handle.position[3] = msg->Position[3];
        tgrArmRobotPtr->location_setting_handle.position[4] = msg->Position[4];
        tgrArmRobotPtr->location_setting_handle.position[5] = msg->Position[5];

        //调用API
        tgrArmRobotPtr->location_setting();
        break;


    case GET_ENCODER: //21   //获取编码器数据
        tgrArmRobotPtr->getEncoders();
        break;

    case GET_POSE:    //22   //获取编码器数据
        tgrArmRobotPtr->getPose();
        usleep(1000);   //1ms
        tgrArmRobotPtr->location_setting();
        break;

    case RETURN_ZERO: //23
        //调用函数
        tgrArmRobotPtr->return_to_zero();
        break;

    case TEMPERAR: //24
        //关闭串口
        tgrArmRobotPtr->usart_stop();
        sleep(1);
        //开启Location上传
        tgrArmRobotPtr->upload_start();
        break;

    default:
        break;
    }
}

//goal回调函数
void TgrArmRobotRos::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    std::cout << "TgrArmRobotRos::executeCB start" << std::endl;
    duration_total = 0;

    //有时可能会需要路径点重排列，当如果是固定赋值，则不需要
    if (ros::ok())
    {
        tgrArmRobotPtr->NumberOfPoints = goal->trajectory.points.size(); //获取路径点数量

        //将路点的终点写入ros_feedback中
        ros_feedback.desired.positions[0] = goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 1].positions[0];
        ros_feedback.desired.positions[1] = goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 1].positions[1];
        ros_feedback.desired.positions[2] = goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 1].positions[2];
        ros_feedback.desired.positions[3] = goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 1].positions[3];
        ros_feedback.desired.positions[4] = goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 1].positions[4];
        ros_feedback.desired.positions[5] = goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 1].positions[5];

        //路径点赋值
        for (int index = 0; index < tgrArmRobotPtr->NumberOfPoints; index++)
        {
            duration_sum = 0;
            numberOfValidDuration = 0;
            //获得各个轴的位置数据数据
            for (int i = 0; i < 6; i++)
            {
                //获取位置信息，也是脉冲信息， 位置 = (物理角度 / PI) * 单位脉冲 + 零点偏移
                tgrArmRobotPtr->trajectory[index].position[i] =
                    (goal->trajectory.points[index].positions[i] * tgrArmRobotPtr->plu2angel[i]) / PI + tgrArmRobotPtr->zeroPlu[i];

                //分段获取速度信息
                if (index == 0)
                {
                    //第一个轨迹点，速度为0
                    durations[i] = 0;
                    tgrArmRobotPtr->trajectory[index].period[i] = 0;
                }
                else if (index == tgrArmRobotPtr->NumberOfPoints - 1)
                {
                    //最后一个轨迹点速度为0，所以要执行这个点，则必须和前一个点的速度保持一致
                    if (goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 2].velocities[i] == 0)
                    {
                        durations[i] = 0;
                        tgrArmRobotPtr->trajectory[index].period[i] = 0;
                    }
                    else
                    {
                        //如果位置未发生变化
                        if (tgrArmRobotPtr->trajectory[index].position[i] == tgrArmRobotPtr->trajectory[index - 1].position[i])
                        {
                            durations[i] = 0;
                            tgrArmRobotPtr->trajectory[index].period[i] = 0;
                        }
                        else
                        {
                            durations[i] =
                                (goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 1].positions[i] -
                                 goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 2].positions[i]) *
                                1000000 / goal->trajectory.points[tgrArmRobotPtr->NumberOfPoints - 2].velocities[i];

                            //和前一个点的速度保持一致
                            tgrArmRobotPtr->trajectory[index].numberOfFullPeriod[i] = tgrArmRobotPtr->trajectory[index - 1].numberOfFullPeriod[i];
                            tgrArmRobotPtr->trajectory[index].restPeriod[i] = tgrArmRobotPtr->trajectory[index - 1].restPeriod[i];
                            tgrArmRobotPtr->trajectory[index].numberOfPeriod[i] = tgrArmRobotPtr->trajectory[index - 1].numberOfPeriod[i];
                            tgrArmRobotPtr->trajectory[index].period[i] = tgrArmRobotPtr->trajectory[index - 1].period[i];

                            numberOfValidDuration++;
                        }
                    }
                }
                else
                {
                    //其余的点，运行时间 = 角度差/速度，  周期 = 运行时间 / 位置差
                    if (goal->trajectory.points[index].velocities[i] == 0)
                    {
                        durations[i] = 0;
                        tgrArmRobotPtr->trajectory[index].period[i] = 0;
                    }
                    else
                    {
                        //如果位置没有发生变化
                        if (tgrArmRobotPtr->trajectory[index].position[i] == tgrArmRobotPtr->trajectory[index - 1].position[i])
                        {
                            durations[i] = 0;
                            tgrArmRobotPtr->trajectory[index].period[i] = 0;
                        }
                        else
                        {
                            //算出当前轴执行完当前点，所需要的时间(单位为us)
                            durations[i] =
                                (goal->trajectory.points[index].positions[i] - goal->trajectory.points[index - 1].positions[i]) * 1000000 / goal->trajectory.points[index].velocities[i];

                            // tgrArmRobotPtr->trajectory[index].period[i] =
                            //     abs((1000000 * PI) / goal->trajectory.points[index].velocities[i] / tgrArmRobotPtr->plu2angel[i]);

                            //记录有效关节数，防止一些关节不进行运动，而导致执行所需要的平均时间偏低
                            numberOfValidDuration++;
                        }
                    }
                }

                //累加出总时间
                duration_sum += abs(durations[i]);

                //std::cout << "positions[" << i <<"] " << this->goal.points[index].positions[i] <<
                //     "  Goal : " << goal->trajectory.points[index].positions[i] << endl;
                //cout << goal->trajectory.points[index].velocities[i] << " ";
            }

            //获得平均时间
            duration_mean = numberOfValidDuration == 0 ? 0 : round(duration_sum / numberOfValidDuration);

            //获取速度
            for (int i = 0; i < 6; i++)
            {
                //只有速度不为0，且发生了位置偏移，才进行脉冲周期计算
                if (goal->trajectory.points[index].velocities[i] != 0 && tgrArmRobotPtr->trajectory[index].position[i] != tgrArmRobotPtr->trajectory[index - 1].position[i])
                {
                    //周期 = 运行时间/脉冲差
                    tgrArmRobotPtr->trajectory[index].period[i] = abs(duration_mean /
                                                                      (tgrArmRobotPtr->trajectory[index].position[i] - tgrArmRobotPtr->trajectory[index - 1].position[i]));

                    //计算出 周期为 period 的周期数
                    tgrArmRobotPtr->trajectory[index].numberOfPeriod[i] = duration_mean %
                                                                          (tgrArmRobotPtr->trajectory[index].position[i] - tgrArmRobotPtr->trajectory[index - 1].position[i]);

                    //对于period超过0xffff，进行一些处理
                    if (tgrArmRobotPtr->trajectory[index].period[i] > 0xffff)
                    { //超过0xffff
                        tgrArmRobotPtr->trajectory[index].numberOfFullPeriod[i] = tgrArmRobotPtr->trajectory[index].period[i] >> 16;
                        tgrArmRobotPtr->trajectory[index].restPeriod[i] = tgrArmRobotPtr->trajectory[index].period[i] & 0xffff;
                    }
                    else
                    { //在0xffff以内
                        tgrArmRobotPtr->trajectory[index].numberOfFullPeriod[i] = 0;
                        tgrArmRobotPtr->trajectory[index].restPeriod[i] = 0;
                    }
                }
            }

            //给出执行一个点所需的时间
            tgrArmRobotPtr->trajectory[index].duration = duration_mean;

            duration_total += duration_mean;
            std::cout << "第 " << index << "个点"
                      << " duration " << duration_mean << "us, "
                      << (double)duration_mean / 1000000 << "s" << endl;
        }

        std::cout << "预计使用" << (double)duration_total / 1000000 << "s" << std::endl;

        //调用tgrArmRobot中的sendTrajectory进行发送数据的操作
#ifdef SUCCESS_INFO
        tgrArmRobotPtr->printTrajectory();
#endif

        tgrArmRobotPtr->sendTrajectory();

        gettimeofday(&tStart, 0);
        //等待状态变化 无需特别高的实时性
        usleep(500000); //至少等待0.5s
        while (!tgrArmRobotPtr->isStopped())
        {
        }

        gettimeofday(&tEnd, 0);
        duration_total_actual = ((tEnd.tv_sec - tStart.tv_sec) * 1000000 + tEnd.tv_usec - tStart.tv_usec);
        //cout << ", 实际使用" << ltime << "us，"

        std::cout << "TgrArmRobotRos::executeCB finished" << endl;

        //检测是否到达最终位置，后期通过一个话题实现紧急取消的功能，使用标志位来通知此处是否被取消
        /*
        for (int i = 0; i < axies; i ++)
        {   
            if (feedback.point.positions[i] != this->goal.points[tgrArmRobotPtr->NumberOfPoints-1].positions[i])
            {
                //动作未完成，反馈抢占性取消
                as->setPreempted();
                return;
            }
        }*/
    }
    //sleep(10);

    //动作完成，反馈结果，设置完成状态
    ros_result.error_code = ros_result.SUCCESSFUL;
    as->setSucceeded(ros_result);

    // std::cout << "路径执行完成" << std::endl;
    // std::cout << "预计使用" << (double)duration_total / 1000000 << "s, 实际使用"
    //           << (double)duration_total_actual / 1000000 << "s" << std::endl;
}

//向ros系统中更新关节状态
void TgrArmRobotRos::jointStateUpdate()
{
    while (ros::ok())
    {
        for (int i = 0; i < 6; i++)
        {
            joint_msg.position[i] = (tgrArmRobotPtr->location.position[i] - tgrArmRobotPtr->zeroPlu[i]) * PI / tgrArmRobotPtr->plu2angel[i];
            ros_feedback.actual.positions[i] = joint_msg.position[i];
        }
        joint_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_msg);

        if (tgrArmRobotPtr->location.state == RUNING)
        {
            ros_feedback.header.stamp = ros::Time::now();
            as->publishFeedback(ros_feedback);
        }

        usleep(100000);
    }
}

//重排序，urdf设计的顺序比较好，可以不用这个
void TgrArmRobotRos::reorder(trajectory_msgs::JointTrajectory trajectory)
{
}



//发送串口
void TgrArmRobotRos::usart_send()
{
    //根据编码器工厂进行通信设计，如果编码器是被动方式，数据放入到缓存中
    char s[18] = "Hello world ros!\n";
    tgrArmRobotPtr->usart_tx_len = 17;
    memcpy(tgrArmRobotPtr->usartTXBuffer, s, tgrArmRobotPtr->usart_tx_len);

    //调用发送
    tgrArmRobotPtr->usart_send();

    sleep(1);
}

//关闭串口
void TgrArmRobotRos::usart_close()
{
    //关闭串口
    tgrArmRobotPtr->usart_stop();

    sleep(1);

    //开启Location上传
    tgrArmRobotPtr->upload_start();
}

//测试
void TgrArmRobotRos::test()
{
    
}