#include "pure_pursuit.h"

#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>
#include <fstream>

#include <nlohmann/json.hpp>

#define PI 3.1415926535

// 构造函数
pure_pursuit::pure_pursuit(ros::NodeHandle &n) : nh(n)
{
    // m_nodeName = ros::this_node::getName();
    n.getParam("comm_hub/vehicle/name", m_vehicleName);
    m_vehicleId = (m_vehicleName[m_vehicleName.size() - 1] - '0' + (m_vehicleName[m_vehicleName.size() - 2] - '0') * 10 - 70);
    n.getParam("comm_hub/vehicle/debug", m_isRun);
    n.getParam("comm_hub/vehicle/speed", currenV);
    n.getParam("comm_hub/vehicle/rad", radtest);
    n.getParam("comm_hub/vehicle/route", m_routeFileName);
    n.getParam("comm_hub/vehicle/angle_error", m_angleError);
    loadRoute(m_routeFileName);

    lhd = 0.333 * currenV + 0.3; // Leading Hold Distance
    // lhd=0.5;
    printf("lhd: %f\n", lhd);
    // 变量初始化,避免还未接受到位置信息时调用越界
    std::vector<geometry_msgs::PoseWithCovarianceStamped> poseMsgs(21);
    CarsInfo.poseMsgs = poseMsgs;

    // 订阅节点
    vrpn_sub = n.subscribe("/motion_capture_pose", 10, &pure_pursuit::vrpnCallback, this);
    // 订阅全部智能车位置信息
    vrpn_sub_all = n.subscribe("/motion_capture_pose_all", 10, &pure_pursuit::vrpnCallback_all, this);
    // 订阅传感器ekf融合数据，包括控制启停信息、速度信息、模式信息
    ctrl_run_sub = n.subscribe("/ctrl_run", 10, &pure_pursuit::ctrl_run_callback, this);
    ctrl_speed_pub = n.subscribe("/ctrl_speed", 10, &pure_pursuit::ctrl_speed_callback, this);
    ctrl_switch_mode_pub = n.subscribe("/ctrl_switch_mode", 10, &pure_pursuit::ctrl_switch_mode_callback, this);

    imu_sub = n.subscribe("/imu", 10, &pure_pursuit::_ImuCallback, this);
    odom_sub = n.subscribe("/odom", 10, &pure_pursuit::_OdomCallback, this);

    // 发布控制节点
    control_pub = n.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 10, true);

    controlPubThread_ = std::thread(&pure_pursuit::_controlPub, this);
}

pure_pursuit::~pure_pursuit()
{
}
// 全部智能车位置信息回调函数
void pure_pursuit::vrpnCallback_all(const serial_ros::PoseMsgs &msg)
{
    // 接受到信息后赋值给全局变量
    CarsInfo = msg;
}

// 处理当前vrpn定位信息
void pure_pursuit::vrpnCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    yaw = yaw + PI / 2 + m_angleError;

    State x0;
    x0.phi = yaw;
    x0.X = msg.pose.pose.position.x;
    x0.Y = msg.pose.pose.position.y;

    if (x0.X < 0 || x0.X > 16 || x0.Y < 0 || x0.Y > 16)
    {
        return;
    }

    if (baseAngle == -10)
    {
        baseAngle = x0.phi;
    }

    baseAngle += x0.phi - imu[0];

    for (size_t i = 0; i < sizeof(imu) / sizeof(imu[0]); i++)
    {
        x0.phi = imu[i];
        x0.X += odom[i] * cos(imu[i]) * 0.02;
        x0.Y += odom[i] * sin(imu[i]) * 0.02;
    }

    m_x = x0.X;
    m_y = x0.Y;
    m_yaw = x0.phi;
}

// 计算车的速度和角速度
void pure_pursuit::ekfStateCallback(const nav_msgs::OdometryConstPtr &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    State x0;
    x0.phi = yaw + PI / 2;
    x0.X = msg->pose.pose.position.x;
    x0.Y = msg->pose.pose.position.y;
    x0.vx = msg->twist.twist.linear.x;
    x0.vy = msg->twist.twist.linear.y;
    x0.r = msg->twist.twist.angular.z;

    // 添加换道逻辑
    whether_change_route(x0.X, x0.Y);

    Eigen::Vector2d targetPos;
    if (false)
    {
    }
    else
    {
        double dist = track_.porjectOnSpline(x0);
        dist += lhd;
        targetPos = track_.getPostion(dist);
        std::cout << "targetPos1" << targetPos[0] <<" , "<< targetPos[1]<<std::endl;
    }

    if (0 > targetPos[0] || targetPos[1] < 0)
    {
        return;
    }

    // 纯跟踪计算
    OutPut u0 = calcPurePursuit(x0, targetPos);

    cmd_vel_linear = u0.V;
    cmd_vel_angular = u0.W;
}

void pure_pursuit::_ImuCallback(const sensor_msgs::Imu &msg)
{
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    yaw += baseAngle;
    if (yaw < -PI)
    {
        yaw += 2 * PI;
    }
    else if (yaw > PI)
    {
        yaw -= 2 * PI;
    }

    m_yaw = yaw;

    for (size_t i = 0; i < sizeof(imu) / sizeof(imu[0]) - 1; i++)
    {
        imu[i] = imu[i + 1];
    }
    imu[sizeof(imu) / sizeof(imu[0]) - 1] = yaw;
}

void pure_pursuit::_OdomCallback(const nav_msgs::Odometry &msg)
{

    m_x += odom[sizeof(odom) / sizeof(odom[0]) - 1] * cos(m_yaw) * 0.02;
    m_y += odom[sizeof(odom) / sizeof(odom[0]) - 1] * sin(m_yaw) * 0.02;

    for (size_t i = 0; i < sizeof(odom) / sizeof(odom[0]) - 1; i++)
    {
        odom[i] = odom[i + 1];
    }
    odom[sizeof(odom) / sizeof(odom[0]) - 1] = msg.twist.twist.linear.x;
}

void pure_pursuit::ctrl_run_callback(const std_msgs::Bool::ConstPtr &msg)
{
    m_isRun = msg->data;
}

void pure_pursuit::ctrl_speed_callback(const std_msgs::UInt32::ConstPtr &msg)
{
    m_speed = msg->data;
}

void pure_pursuit::ctrl_switch_mode_callback(const std_msgs::UInt32::ConstPtr &msg)
{
    m_mode = msg->data;
}

/*
    这个函数用于选择避障策略。
    0：实验二的默认避障策略（这种策略很容易越界）
    1：懒惰避障（直接停车）
    2：减速避障，发现障碍物之后开始减速，小于碰撞距离之后直接停车
    3：变道超车避障，发现障碍物之后向左变道超车
*/
OutPut pure_pursuit::barrier_avoidance_mode(int choice, OutPut u0, double extreme_danger_flag, double dis, double current_yaw, double x, double y, double target_x, double target_y)
{
    OutPut u1 = u0;

    if (choice == 0)
    {
        // 实验二的默认避障策略
        u1.W = radtest;
    }
    else if (choice == 1)
    {
        // 懒惰避障
        u1.V = 0;
        u1.W = 0;
    }
    else if (choice == 2)
    {
        // 减速避障
        if (extreme_danger_flag)
        {
            u1.V = -0.5;
            u1.W = 0;
        }
        else
        {
            // 此时dis介于1到2之间，减1是为了让dis介于0到1之间
            u1.V = currenV * (dis - 1);
            if(u1.V<0.05) u1.V=-0.5; 
        }
    }
    else if (choice == 3)
    {
        // 向左变道超车避障逻辑
        if (extreme_danger_flag)
        {
            u1.V = 0;
        }
        else if (dis < 2.0 && dis >= 1.0)
        {
            u1.V = currenV * (dis - 1); // 减速

            // 计算新的目标点，向左偏移一定距离
            double offset_distance = 0.5; // 左偏移一定距离
            // 具体加减号需要现场看看
            double new_target_x = target_x + offset_distance * sin(current_yaw);
            double new_target_y = target_y - offset_distance * cos(current_yaw);

            // 重新计算到新的目标点的角度和距离
            double ld = sqrt(pow(new_target_x - x, 2) + pow(new_target_y - y, 2));
            double sita = atan2(new_target_y - y, new_target_x - x);
            double alpha = sita - current_yaw;

            if (alpha < -PI)
            {
                alpha += 2 * PI;
            }
            else if (alpha > PI)
            {
                alpha -= 2 * PI;
            }

            u1.W = currenV * 2 * sin(alpha) / ld; // 动态计算角速度
        }
    }

    return u1;
}

// 此函数的作用是判断是根据当前位置需要变换路径
// 添加时间：2024-6-24
void pure_pursuit::whether_change_route(double x_curr, double y_curr){
    // 判断受否需要换道的锚点，现场可以再调整
    double x_anchor = 2.567567568, y_anchor = 0.45946;

    // 这里临界的角度和距离也需要现场调整
    if(PointInSector3(x_curr, y_curr, x_anchor, y_anchor, 1.0, 30.0)){
        // 需要换道
        // 重新加载路径
        loadRoute("/home/firefly/class/Experiment3/src/Pure_Pursuit/routes/7.json");
    }
}

void pure_pursuit::_controlPub()
{
    double yaw;
    geometry_msgs::Quaternion quat;

    OutPut u2;
    u2.W=0;

    while (1)
    {
        State x0;
        x0.phi = m_yaw;
        x0.X = m_x;
        x0.Y = m_y;
         std::cout<<"xy"<<x0.X<<","<<x0.Y<<std::endl;

        // 添加换道逻辑
        whether_change_route(x0.X, x0.Y);

        Eigen::Vector2d targetPos;
        if (false)
        {
            std::cout<<"m_mode"<<std::endl;
        }
        else
        {
            double dist = track_.porjectOnSpline(x0);
            dist += lhd;
            targetPos = track_.getPostion(dist);
            std::cout << "targetPos1" << targetPos[0] <<" , "<< targetPos[1]<<std::endl;
        }

        if (0 > targetPos[0] || targetPos[1] < 0)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            return;
        }

        // 全局定位信息处理
        double flag = 0;
        double extreme_danger_flag = 0;
        double dis = 16;
        // geometry_msgs::Quaternion quat=msg.poseMsgs[m_vehicleId].pose.pose.orientation; 
        double yaw= m_yaw;//yaw 
        for (int i = 0; i < 20; i++)
        {   // Car70 ~ Car89
            if (i == m_vehicleId)
                continue;                                                                 // 不和本车判断
            double Pose_x1 = CarsInfo.poseMsgs[i].pose.pose.position.x;                   // 他车 x 坐标
            double Pose_y1 = CarsInfo.poseMsgs[i].pose.pose.position.y;                   // 他车 y 坐标
           
            double Car_yaw = quaternionToYaw(CarsInfo.poseMsgs[i].pose.pose.orientation); // 他车 yaw 角
            if (Pose_x1 < 0 || Pose_x1 > 16 || Pose_y1 < 0 || Pose_y1 > 16)
                continue; // 他车在场地外不判断

            double ang=30;
            if(u2.W*u2.W>1) ang=60;

            if (PointInSector2(x0.X, x0.Y, Pose_x1, Pose_y1, x0.phi,2,ang))
            { // 判断是否被该车阻挡了
                flag = 1;
                if (PointInSector2(x0.X, x0.Y, Pose_x1, Pose_y1, x0.phi, 1, ang))
                { // 判断是否处于极度危险区域
                    extreme_danger_flag = 1;
                }
                else
                {
                    dis = std::min(dis, hypot(x0.X - Pose_x1, x0.Y - Pose_y1));
                }
            }
        }

        // 纯跟踪计算
        OutPut u0 = calcPurePursuit(x0, targetPos);
        std::cout << "targetPos: " <<targetPos[0]<<"   " <<targetPos[1]<< std::endl;
        u2 = u0;

        // 需要修改这里的避障操作
        if (flag)
        {
            OutPut u1 = barrier_avoidance_mode(2, u0, extreme_danger_flag, dis, x0.phi, x0.X, x0.Y, targetPos[0], targetPos[1]); // 使用超车避障策略

            cmd_vel_linear = u1.V;
            cmd_vel_angular = u1.W;
        }
        else
        {
            cmd_vel_linear = u0.V;
            cmd_vel_angular = u0.W;
        }

        // 生成控制指令
        ros::Time curTime = ros::Time::now();
        geometry_msgs::TwistStamped twist_stam;
        twist_stam.header.seq = seq++;
        twist_stam.header.stamp = curTime;
        twist_stam.header.frame_id = "base_link";

        geometry_msgs::Vector3 linear;
        linear.x = cmd_vel_linear;
        linear.y = 0;
        linear.z = 0;
        geometry_msgs::Vector3 angular;
        angular.x = 0;
        angular.y = 0;
        angular.z = cmd_vel_angular;

        if (!m_isRun)
        {
            linear.x = 0;
            angular.z = 0;
        }

        twist_stam.twist.linear = linear;
        twist_stam.twist.angular = angular;

        control_pub.publish(twist_stam);
        if (linear.x == -0.5)
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        // std::cout << linear << " " << angular << std::endl;
    }
}

bool pure_pursuit::PointInSector(double x, double y, double x1, double y1, double yaw)
{
    double rad = atan2(y1 - y, x1 - x);
    double dis = hypot(x1 - x, y1 - y);

    double rad1 = fabs(rad - yaw);
    if (rad1 > PI)
    {
        rad1 = 2 * PI - rad1;
    }
    if ((rad1 / PI * 180 < 30) && dis <= 2.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

bool pure_pursuit::PointInSector2(double x, double y, double x1, double y1, double yaw, double dis1, double angle1)
{
    double rad = atan2(y1 - y, x1 - x);
    double dis = hypot(x1 - x, y1 - y);
    double rad1 = fabs(rad - yaw);
    if (rad1 > PI)
    {
        rad1 = 2 * PI - rad1;
    }
    if ((rad1 / PI * 180 < angle1) && dis <= dis1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

// 新加了一个不用传入yaw的版本
bool pure_pursuit::PointInSector3(double x, double y, double x1, double y1, double dis1, double angle1)
{
    double rad = atan2(y1 - y, x1 - x);
    double dis = hypot(x1 - x, y1 - y);
    double rad1 = fabs(rad - m_yaw);
    if (rad1 > PI)
    {
        rad1 = 2 * PI - rad1;
    }
    if ((rad1 / PI * 180 < 30) && dis <= 2.0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

OutPut pure_pursuit::calcPurePursuit(const State &state, Eigen::Vector2d targetPos)
{
    OutPut u0;
    double ld = sqrt(pow(targetPos[0] - state.X, 2) + pow(targetPos[1] - state.Y, 2));
    double sita = atan2(targetPos[1] - state.Y, targetPos[0] - state.X);
    double alpha = sita - state.phi;
    if (alpha < -PI)
    {
        alpha += 2 * PI;
    }
    else if (alpha > PI)
    {
        alpha -= 2 * PI;
    }

    u0.V = currenV;
    u0.W = currenV * 2 * sin(alpha) / ld;

    u0.X = targetPos[0];
    u0.Y = targetPos[1];

    return u0;
}

void pure_pursuit::loadRoute(std::string filePath)
{
    ROS_INFO("loading route");
    std::ifstream iTrack(filePath);
    nlohmann::json jsonTrack;
    iTrack >> jsonTrack;
    std::vector<double> X = jsonTrack["X"];
    std::vector<double> Y = jsonTrack["Y"];
    Eigen::VectorXd vdX = Eigen::Map<Eigen::VectorXd>(X.data(), X.size());
    Eigen::VectorXd vdY = Eigen::Map<Eigen::VectorXd>(Y.data(), Y.size());
    track_.gen2DSpline(vdX, vdY);
}
double pure_pursuit::quaternionToYaw(const geometry_msgs::Quaternion &quat)
{ // 四元数转角度
    // 将geometry_msgs::Quaternion转换为tf2::Quaternion
    tf2::Quaternion tf_quat;
    tf2::fromMsg(quat, tf_quat);

    // 使用tf2库来执行旋转矩阵计算
    tf2::Matrix3x3 matrix(tf_quat);

    // 获取Yaw角（绕Z轴的旋转）
    double yaw, pitch, roll;
    matrix.getRPY(roll, pitch, yaw);
    yaw += PI / 2; // 改为以x轴为0方向
    if (yaw > PI)
        yaw = yaw - 2 * PI;
    return yaw;
}