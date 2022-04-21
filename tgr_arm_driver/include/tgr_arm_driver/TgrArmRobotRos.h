#include "TgrArmRobot.h"

//来自 自定义消息文件 转换成的头文件
#include "tgr_arm_driver/ExtraFeatures.h"



//机器人指针实例
extern TgrArmRobot * tgrArmRobotPtr;

//太长了，简化一下名字
typedef actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> Server;


class TgrArmRobotRos
{
public:
    friend void jointStateUpdate(TgrArmRobotRos * p);

    //构造函数
    TgrArmRobotRos();

    ~TgrArmRobotRos();

    //extraFeatures功能回调函数
    void extraFeaturesCB(const tgr_arm_driver::ExtraFeaturesConstPtr &msg);

    //goal回调函数
    void executeCB(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

	//向ros系统中更新关节状态
	void jointStateUpdate();

	//重排序，urdf设计的顺序比较好，可以不用这个
	void reorder(trajectory_msgs::JointTrajectory trajectory);

    //发送串口
    void usart_send();

    //关闭串口
    void usart_close();

    //测试专用
    void test();

private:




    //执行时间us
    int durations[6];
    //平均各个关节执行完毕所需的时间 us
    int duration_mean;
    //各个关节执行时间之和
    int duration_sum;
    //非0的关节，执行当前点时，周期不为0的关节个数
    int numberOfValidDuration;
    //轨迹执行完毕所花费的时间
    int duration_total;
    int duration_total_actual;
    

    //ROS，ROS部分的关节数，要和模型保持一致
    //句柄实例
    ros::NodeHandle nh;

    //action名称
    string action_name;

    //定义action服务端实例，只有指针才能方便的在初始化器中初始化
    Server * as;
    //actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> * as;

    //反馈实例
    control_msgs::FollowJointTrajectoryFeedback ros_feedback;

    //用来反馈action目标的执行情况，客户端由此可以得知服务端是否执行成功了
    control_msgs::FollowJointTrajectoryResult ros_result; 

    //关节状态发布者 消息实例
    ros::Publisher joint_pub;
    sensor_msgs::JointState joint_msg;

    //功能状态接收者，可以触发回调函数
    ros::Subscriber extra_features_sub;
    tgr_arm_driver::ExtraFeatures extra_features_msg;


    //性能定时器
    struct timeval tStart;
    struct timeval tEnd;
};