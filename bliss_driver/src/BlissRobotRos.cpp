#include "BlissRobotRos.h"

//机械臂实例
//BlissRobot * blissRobotPtr = new BlissRobot("192.168.31.215", 8080);
BlissRobot *blissRobotPtr = new BlissRobot();

//BlissRobot 监听友元函数
void listening(BlissRobot *p)
{
    std::cout << "void listening \n";
    p->listening();
}

BlissRobotRos::BlissRobotRos()
{
    //句柄实例
    ros::NodeHandle nh;

    //动作名称
    action_name = "/bliss/bliss_controller/follow_joint_trajectory";

    //初始化关节变量
    joint_msg.name.resize(6);
    joint_msg.position.resize(6);
    joint_msg.header.frame_id = "/bliss";

    //初始化ros_feedback
    ros_feedback.header.frame_id = "/bliss";
    ros_feedback.desired.positions.resize(6);
    ros_feedback.actual.positions.resize(6);

    //关节命名
    joint_msg.name[0] = "joint1";
    joint_msg.name[1] = "joint2";
    joint_msg.name[2] = "joint3";
    joint_msg.name[3] = "joint4";
    joint_msg.name[4] = "joint5";
    joint_msg.name[5] = "joint6";

    //各个关节执行完毕所需的时间
    memset(durations, 0, sizeof(durations));
    duration_sum = 0;

    //功能
    extra_features_msg.Tag = 0;
    extra_features_msg.Position.resize(6);

    //重要参数
    //旋转+180°(+3.1415926)，需要的节拍
    plu2angel[0] = -15600;
    plu2angel[1] = -12700;
    plu2angel[2] = -16000;
    plu2angel[3] = 9100;
    plu2angel[4] = 6800;
    plu2angel[5] = 3200;

    //零点参数

    zeroPlu[0] = 0;
    zeroPlu[1] = -5600;
    zeroPlu[2] = 6600;
    zeroPlu[3] = 0;
    zeroPlu[4] = 3200;
    zeroPlu[5] = 0;

    // plu2angel[0] = -31200;
    // plu2angel[1] = -25400;
    // plu2angel[2] = -32000;
    // plu2angel[3] = 18200;
    // plu2angel[4] = 13600;
    // plu2angel[5] = 6400;

    // //零点参数

    // zeroPlu[0] = 0;
    // zeroPlu[1] = -11200;
    // zeroPlu[2] = 13200;
    // zeroPlu[3] = 0;
    // zeroPlu[4] = 6800;
    // zeroPlu[5] = 0;

    //启动
    blissRobotPtr->startConstruction();

    //监听机械臂
    std::thread t_listening = std::thread(listening, blissRobotPtr);

    //关节发布者初始化
    joint_pub = nh.advertise<sensor_msgs::JointState>("/bliss/joint_states", 1);

    //服务器初始化
    as = new Server(nh, action_name, boost::bind(&BlissRobotRos::executeCB, this, _1), false);

    //服务器开启
    as->start();

    //功能订阅者初始化
    extra_features_sub = nh.subscribe("/bliss/extraFeatures", 1, &BlissRobotRos::extraFeaturesCB, this);

    ros::AsyncSpinner spinner(1);
    spinner.start();

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

BlissRobotRos::~BlissRobotRos()
{
}

//extraFeatures功能回调函数
void BlissRobotRos::extraFeaturesCB(const bliss_driver::ExtraFeaturesConstPtr &msg)
{
    //获取消息标签
    extra_features_msg.Tag = msg->Tag;

    switch (msg->Tag)
    {
    case STOPPED:
        //紧急制动
        blissRobotPtr->robot_stop();
        break;

    case PWM_START:
        //获取数据
        blissRobotPtr->pwm_handle.PSC = msg->PSC;
        blissRobotPtr->pwm_handle.ARR = msg->ARR;
        blissRobotPtr->pwm_handle.CCR1 = msg->CCR1;
        blissRobotPtr->pwm_handle.PluseCount = msg->PluseCount;
        //发送pwm数据
        blissRobotPtr->pwm_start();
        break;

    case PWM_STOP:
        //关闭pwm
        blissRobotPtr->pwm_stop();
        break;

    case RETURN:
        //调用函数
        return_to_zero();
        break;
    
    case 0:
        //关闭串口
        blissRobotPtr->usart_stop();

        //sleep(1);

        //开启Location上传
        blissRobotPtr->upload_start();
        break;

    case PIN0_ON:
        blissRobotPtr->pin0_on();
        break;

    case PIN0_OFF:
        blissRobotPtr->pin0_off();
        break;

    case PIN1_ON:
        blissRobotPtr->pin1_on();
        break;

    case PIN1_OFF:
        blissRobotPtr->pin1_off();
        break;

    case TOGGLE_ENABLE_PINS:
        blissRobotPtr->toggle_enable_pins();
        break;

    case LOCATION_SETTING:
        //设置当前角度脉冲数值,必须要在机械臂完全停止运动的时候使用这个功能
        blissRobotPtr->location_setting_handle.state = LOCATION_SETTING;
        
        //6轴
        blissRobotPtr->location_setting_handle.position[0] = msg->Position[0];
        blissRobotPtr->location_setting_handle.position[1] = msg->Position[1];
        blissRobotPtr->location_setting_handle.position[2] = msg->Position[2];
        blissRobotPtr->location_setting_handle.position[3] = msg->Position[3];
        blissRobotPtr->location_setting_handle.position[4] = msg->Position[4];
        blissRobotPtr->location_setting_handle.position[5] = msg->Position[5];

        //调用API
        blissRobotPtr->location_setting();
        break;
    default:
        break;
    }
}

//goal回调函数
void BlissRobotRos::executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    std::cout << "BlissRobotRos::executeCB start" << std::endl;
    duration_total = 0;

    //有时可能会需要路径点重排列，当如果是固定赋值，则不需要
    if (ros::ok())
    {
        blissRobotPtr->NumberOfPoints = goal->trajectory.points.size(); //获取路径点数量

        //将路点的终点写入ros_feedback中
        ros_feedback.desired.positions[0] = goal->trajectory.points[blissRobotPtr->NumberOfPoints - 1].positions[0];
        ros_feedback.desired.positions[1] = goal->trajectory.points[blissRobotPtr->NumberOfPoints - 1].positions[1];
        ros_feedback.desired.positions[2] = goal->trajectory.points[blissRobotPtr->NumberOfPoints - 1].positions[2];
        ros_feedback.desired.positions[3] = goal->trajectory.points[blissRobotPtr->NumberOfPoints - 1].positions[3];
        ros_feedback.desired.positions[4] = goal->trajectory.points[blissRobotPtr->NumberOfPoints - 1].positions[4];
        ros_feedback.desired.positions[5] = goal->trajectory.points[blissRobotPtr->NumberOfPoints - 1].positions[5];

        //路径点赋值
        for (int index = 0; index < blissRobotPtr->NumberOfPoints; index++)
        {
            duration_sum = 0;
            numberOfValidDuration = 0;
            //获得各个轴的位置数据数据
            for (int i = 0; i < 6; i++)
            {
                //获取位置信息，也是脉冲信息， 位置 = (物理角度 / PI) * 单位脉冲 + 零点偏移
                blissRobotPtr->trajectory[index].position[i] =
                    (goal->trajectory.points[index].positions[i] * plu2angel[i]) / PI + zeroPlu[i];

                //分段获取速度信息
                if (index == 0)
                {
                    //第一个轨迹点，速度为0
                    durations[i] = 0;
                    blissRobotPtr->trajectory[index].period[i] = 0;
                }
                else if (index == blissRobotPtr->NumberOfPoints - 1)
                {
                    //最后一个轨迹点速度为0，所以要执行这个点，则必须和前一个点的速度保持一致
                    if (goal->trajectory.points[blissRobotPtr->NumberOfPoints - 2].velocities[i] == 0)
                    {
                        durations[i] = 0;
                        blissRobotPtr->trajectory[index].period[i] = 0;
                    }
                    else
                    {
                        //如果位置未发生变化
                        if (blissRobotPtr->trajectory[index].position[i] == blissRobotPtr->trajectory[index - 1].position[i])
                        {
                            durations[i] = 0;
                            blissRobotPtr->trajectory[index].period[i] = 0;
                        }
                        else
                        {
                            durations[i] =
                                (goal->trajectory.points[blissRobotPtr->NumberOfPoints - 1].positions[i] -
                                 goal->trajectory.points[blissRobotPtr->NumberOfPoints - 2].positions[i]) *
                                1000000 / goal->trajectory.points[blissRobotPtr->NumberOfPoints - 2].velocities[i];

                            //和前一个点的速度保持一致
                            blissRobotPtr->trajectory[index].numberOfFullPeriod[i] = blissRobotPtr->trajectory[index - 1].numberOfFullPeriod[i];
                            blissRobotPtr->trajectory[index].restPeriod[i] = blissRobotPtr->trajectory[index - 1].restPeriod[i];
                            blissRobotPtr->trajectory[index].numberOfPeriod[i] = blissRobotPtr->trajectory[index - 1].numberOfPeriod[i];
                            blissRobotPtr->trajectory[index].period[i] = blissRobotPtr->trajectory[index - 1].period[i];


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
                        blissRobotPtr->trajectory[index].period[i] = 0;
                    }
                    else
                    {
                        //如果位置没有发生变化
                        if (blissRobotPtr->trajectory[index].position[i] == blissRobotPtr->trajectory[index - 1].position[i])
                        {
                            durations[i] = 0;
                            blissRobotPtr->trajectory[index].period[i] = 0;
                        }
                        else
                        {
                            //算出当前轴执行完当前点，所需要的时间(单位为us)
                            durations[i] =
                                (goal->trajectory.points[index].positions[i] - goal->trajectory.points[index - 1].positions[i]) * 1000000 / goal->trajectory.points[index].velocities[i];

                            // blissRobotPtr->trajectory[index].period[i] =
                            //     abs((1000000 * PI) / goal->trajectory.points[index].velocities[i] / plu2angel[i]);

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
                if (goal->trajectory.points[index].velocities[i] != 0 && blissRobotPtr->trajectory[index].position[i] != blissRobotPtr->trajectory[index - 1].position[i])
                {
                    //周期 = 运行时间/脉冲差
                    blissRobotPtr->trajectory[index].period[i] = abs(duration_mean /
                                                                     (blissRobotPtr->trajectory[index].position[i] - blissRobotPtr->trajectory[index - 1].position[i]));

                    //计算出 周期为 period 的周期数
                    blissRobotPtr->trajectory[index].numberOfPeriod[i] = duration_mean %
                                                                         (blissRobotPtr->trajectory[index].position[i] - blissRobotPtr->trajectory[index - 1].position[i]);

                    //对于period超过0xffff，进行一些处理
                    if (blissRobotPtr->trajectory[index].period[i] > 0xffff)
                    { //超过0xffff
                        blissRobotPtr->trajectory[index].numberOfFullPeriod[i] = blissRobotPtr->trajectory[index].period[i] >> 16;
                        blissRobotPtr->trajectory[index].restPeriod[i] = blissRobotPtr->trajectory[index].period[i] & 0xffff;
                    }
                    else
                    { //在0xffff以内
                        blissRobotPtr->trajectory[index].numberOfFullPeriod[i] = 0;
                        blissRobotPtr->trajectory[index].restPeriod[i] = 0;
                    }
                }
            }

            //给出执行一个点所需的时间
            blissRobotPtr->trajectory[index].duration = duration_mean;

            duration_total += duration_mean;
            std::cout << "第 " << index << "个点"
                      << " duration " << duration_mean << "us, "
                      << (double)duration_mean / 1000000 << "s" << endl;
        }

        std::cout << "预计使用" << (double)duration_total / 1000000 << "s" << std::endl;

        //调用blissRobot中的sendTrajectory进行发送数据的操作
        blissRobotPtr->printTrajectory();

        blissRobotPtr->sendTrajectory();

        gettimeofday(&tStart, 0);
        //等待状态变化 无需特别高的实时性
        usleep(500000); //至少等待0.5s
        while (!blissRobotPtr->isStopped())
        {
        }

        gettimeofday(&tEnd, 0);
        duration_total_actual = ((tEnd.tv_sec - tStart.tv_sec) * 1000000 + tEnd.tv_usec - tStart.tv_usec);
        //cout << ", 实际使用" << ltime << "us，"

        std::cout << "BlissRobotRos::executeCB finished" << endl;

        //检测是否到达最终位置，后期通过一个话题实现紧急取消的功能，使用标志位来通知此处是否被取消
        /*
        for (int i = 0; i < axies; i ++)
        {   
            if (feedback.point.positions[i] != this->goal.points[blissRobotPtr->NumberOfPoints-1].positions[i])
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

    std::cout << "路径执行完成" << std::endl;
    std::cout << "预计使用" << (double)duration_total / 1000000 << "s, 实际使用"
              << (double)duration_total_actual / 1000000 << "s" << std::endl;
}

//向ros系统中更新关节状态
void BlissRobotRos::jointStateUpdate()
{
    while (ros::ok())
    {
        for (int i = 0; i < 6; i++)
        {
            joint_msg.position[i] = (blissRobotPtr->location.position[i] - zeroPlu[i]) * PI / plu2angel[i];
            ros_feedback.actual.positions[i] = joint_msg.position[i];
        }
        joint_msg.header.stamp = ros::Time::now();
        joint_pub.publish(joint_msg);

        if (blissRobotPtr->location.state == RUNING)
        {
            ros_feedback.header.stamp = ros::Time::now();
            as->publishFeedback(ros_feedback);
        }

        usleep(100000);
    }
}

//重排序，urdf设计的顺序比较好，可以不用这个
void BlissRobotRos::reorder(trajectory_msgs::JointTrajectory trajectory)
{
}

//机械臂根据编码器数据归零，由于有不同编码器，此部分通常由用户完成
void BlissRobotRos::return_to_zero()
{
    //先关闭Location上传
    blissRobotPtr->upload_stop();

    //sleep(1);

    //获取编码器数据
    blissRobotPtr->usart_start();

    //sleep(1);

    //根据编码器工厂进行通信设计，如果编码器是被动方式，数据放入到缓存中
    char s[18] = "Hello world ros!\n";
    blissRobotPtr->usart_tx_len = 17;
    memcpy(blissRobotPtr->usartTXBuffer, s, blissRobotPtr->usart_tx_len);

    //调用发送
    blissRobotPtr->usart_send();

    sleep(1);

    //等待获得编码器数据，然后进行执行，慢慢调整数据到零点位置
    //
    //
    //
    //

    /*
    //设置一个执行点
    blissRobotPtr->NumberOfPoints = 1;

    //先进行清零
    memset(&blissRobotPtr->trajectory[0], 0, PointSize);

    //设置执行位置
    // blissRobotPtr->trajectory[0].position[0] = 0;
    // blissRobotPtr->trajectory[0].position[1] = 0;
    // blissRobotPtr->trajectory[0].position[2] = 0;
    // blissRobotPtr->trajectory[0].position[3] = 0;
    // blissRobotPtr->trajectory[0].position[4] = 0;
    // blissRobotPtr->trajectory[0].position[5] = 0;
    // blissRobotPtr->trajectory[0].position[6] = 0;
    // blissRobotPtr->trajectory[0].position[7] = 0;

    //无需运动的关节，将于10ms关闭
    blissRobotPtr->trajectory[0].period = 10000;        

    //设置执行速度
    blissRobotPtr->trajectory[0].duration[0] = 5000;
    blissRobotPtr->trajectory[0].duration[1] = 5000;
    blissRobotPtr->trajectory[0].duration[2] = 5000;
    blissRobotPtr->trajectory[0].duration[3] = 5000;
    blissRobotPtr->trajectory[0].duration[4] = 5000;
    blissRobotPtr->trajectory[0].duration[5] = 5000;
    blissRobotPtr->trajectory[0].duration[6] = 5000;
    blissRobotPtr->trajectory[0].duration[7] = 5000;




    //调用blissRobot中的sendTrajectory进行发送数据的操作
    blissRobotPtr->printTrajectory();

    //发送数据
    blissRobotPtr->sendTrajectory();
    */

    //获取编码器数据





    //关闭串口
    //blissRobotPtr->usart_stop();

    //sleep(1);

    //开启Location上传
    //blissRobotPtr->upload_start();
}