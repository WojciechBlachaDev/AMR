#include "ros/ros.h"
#include <string>
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include "sick_lidar_localization/LocalizationControllerResultMessage0502.h"
#include <sstream>
#include <eigen3/Eigen/Dense>
using namespace std;
using std::vector;
using Eigen::MatrixXd;
using Eigen::VectorXd;
ros::Publisher odom_data_pub;
ros::Publisher odom_data_pub_quat;
ros::Publisher sickLidarPose2D_pub;
ros::Publisher odomPose2D_pub;
nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;
nav_msgs::Odometry sickLidarOdom;
std_msgs::Int64 PWM_DutyCycle;
std_msgs::Int64 ServoTick;
geometry_msgs::Pose2D odomPose2D;
geometry_msgs::Pose2D sickLidarPose2D;
tf::TransformBroadcaster *tb;
Eigen::Matrix3d A;
Eigen::Matrix3d A_T;
Eigen::MatrixXd B_k_1(3, 2);			
Eigen::Matrix3d H;
Eigen::Matrix3d H_T; 		
Eigen::Matrix3d R_k;
Eigen::Matrix3d Q_k;
Eigen::Matrix3d K_k;
Eigen::Matrix3d S_k;
Eigen::Matrix3d invS_k;
Eigen::Matrix3d P_k;
Eigen::Vector3d x_k;
Eigen::Vector3d x_k1;
Eigen::Vector3d x_dot;
Eigen::Vector3d y_k;
Eigen::Vector2d u_k_1;
Eigen::Vector3d w_k;
Eigen::Vector3d v_k; 

const double initialX = 0.0;
const double initialY = 0.0;
const double initialFi = 0.0;
const double PI = 3.141592;
const double WHEEL_RADIUS = 0.105;
const double BASE = 1.195; 
double DeltaTime = 0, TimeOld = 0, Czas=0;
double Vw=0, V=0, Psi = 0, Fi_dot=0, Fi=0, Lidar_x=0, Lidar_y=0, Lidar_Fi=0;
double x=0, y=0, fi=0, Wozek_Vw=0, Wozek_Psi=0;
double d_t=0.0;
long time_i = 0;
int ii = 1;
int iii = 0;
bool initialPoseRecieved = false;

void set_initial_2d(const geometry_msgs::PoseStamped &rvizClick) 
{
    odomOld.pose.pose.position.x = rvizClick.pose.position.x;
    odomOld.pose.pose.position.y = rvizClick.pose.position.y;
    odomOld.pose.pose.orientation.z = rvizClick.pose.orientation.z;
    initialPoseRecieved = true;
}

void GetVw(const std_msgs::Float32& Velocity) 
{
    Wozek_Vw = Velocity.data;
}

void GetPsi(const std_msgs::Float32& Angle) 
{
    Wozek_Psi = Angle.data;
}

void LidarLocPose(const sick_lidar_localization::LocalizationControllerResultMessage0502 &msg) 
{
    Lidar_x = msg.x/1000.0;
    Lidar_y = msg.y/1000.0;
    Lidar_Fi = ((msg.heading/1000.0) * PI)/180.0;
    sickLidarPose2D.x = Lidar_x;
    sickLidarPose2D.y = Lidar_y;
    sickLidarPose2D.theta = Lidar_Fi;
    sickLidarPose2D_pub.publish(sickLidarPose2D);
}

void publish_quat() 
{ 
    tf2::Quaternion q;
    q.setRPY(0, 0, odomNew.pose.pose.orientation.z);
    nav_msgs::Odometry quatOdom;
    quatOdom.header.stamp = odomNew.header.stamp;
    quatOdom.header.frame_id = "odom";
    quatOdom.child_frame_id = "base_link";
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
    quatOdom.pose.pose.orientation.x = q.x();
    quatOdom.pose.pose.orientation.y = q.y();
    quatOdom.pose.pose.orientation.z = q.z();
    quatOdom.pose.pose.orientation.w = q.w();
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;
    for(int i = 0; i<36; i++) 
    {
        if(i == 0 || i == 7 || i == 14) 
        {
            quatOdom.pose.covariance[i] = .01;
        }
        else if (i == 21 || i == 28 || i== 35) 
        {
            quatOdom.pose.covariance[i] += 0.1;
        }
        else 
        {
            quatOdom.pose.covariance[i] = 0;
        }
    }
    odom_data_pub_quat.publish(quatOdom);
}

void update_trans() 
{
    tb = new tf::TransformBroadcaster();
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_footprint";
    transformStamped.header.stamp = ros::Time::now(); 
    transformStamped.transform.translation.x = odomNew.pose.pose.position.x;
    transformStamped.transform.translation.y = odomNew.pose.pose.position.y; 
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = tf::createQuaternionMsgFromYaw(sickLidarPose2D.theta);
    tb->sendTransform(transformStamped);	
}

void update_odom() 
{
    odomNew.header.stamp = ros::Time::now();
    if (ii == 1)	
    {
        TimeOld = odomNew.header.stamp.toSec();  
        ii=2; 
    }
    else	
    {

    }
    DeltaTime = (odomNew.header.stamp.toSec() - TimeOld);
    TimeOld = odomNew.header.stamp.toSec();
    d_t=DeltaTime;
    if ((d_t != 0)	&& ((Lidar_x != 0) || (Lidar_y != 0)|| (Lidar_Fi != 0)))	  
    {
        u_k_1(0,0)	= 	Wozek_Vw * cos(Wozek_Psi);			
        u_k_1(1,0)	= Wozek_Vw * (sin(Wozek_Psi)/BASE);		
        y_k	<<	Lidar_x,	Lidar_y,	Lidar_Fi;
        Fi = Fi + (d_t * u_k_1(1,0));
        B_k_1	<<		cos(Fi)*d_t,	0,
                    sin(Fi)*d_t,	0,
                    0,	          d_t;
        x_k = A * x_k + B_k_1 * u_k_1;
        A_T = A;
        P_k =  A * P_k * A_T + Q_k;
        H_T = H;
        S_k = H * P_k * H_T + R_k;
        invS_k = S_k.inverse();
        K_k = P_k * H_T * invS_k;
        P_k =  P_k - K_k * H * P_k;
        x_k = x_k + K_k * (y_k - H * x_k);
        ROS_DEBUG_STREAM("Lidar_x="<< Lidar_x << "  x_k= "<< x_k(0,0)<< " # Lidar_y="<< Lidar_y<< "  y_k="<< x_k(1,0) << " # Lidar_Fi="<< Lidar_Fi << " Fi_k="<< x_k(2,0));
        if (iii == 0) 
        {
            x_k1 = x_k;
            iii=2;
            x_dot	<<	0.0,	0.0,	0.0;
            ROS_DEBUG_STREAM("iii == 0");
        }
        else 
        {
            ROS_DEBUG_STREAM("iii == 2");
            x_dot(0,0) = (x_k(0,0) - x_k1(0,0) ) / d_t; 
            x_dot(1,0) = (x_k(1,0) - x_k1(1,0) ) / d_t; 

            if ((x_k(2,0) * x_k1(2,0)<0) && (x_dot(2,0)>0))   
            {
                x_dot(2,0) = ( 2 * PI - abs(x_k(2,0) - x_k1(2,0)))  / d_t;
            }
            else if ((x_k(2,0) * x_k1(2,0)<0) && (x_dot(2,0)<0)) 
            {
            x_dot(2,0) = -1 * (( 2 * PI - abs(x_k(2,0) - x_k1(2,0)))  / d_t);
            }
            else 
            {
                x_dot(2,0) = (x_k(2,0) - x_k1(2,0))  / d_t;
            }
        }
        ROS_DEBUG_STREAM("d_t= "<< d_t);
        ROS_DEBUG_STREAM("x_k(0,0)= "<< x_k(0,0)<<" x_k(1,0)"<< x_k(1,0) <<" x_k(2,0)= "<<x_k(2,0));
        ROS_DEBUG_STREAM("x_k1(0,0)= "<< x_k1(0,0)<<" x_k1(1,0)"<< x_k1(1,0) <<" x_k1(2,0)= "<<x_k1(2,0));
        ROS_DEBUG_STREAM("x_dot="<<  x_dot(0,0)<< " # y_dot="<< x_dot(1,0)<< "  # Fi_dot="<< x_dot(2,0));
        x_k1 = x_k;
    }
    else
    {

    }
    float alfa = 0.8;
    double  X_dot = alfa * X_dot + (1 - alfa) * x_dot(0,0);
    double  Y_dot = alfa * Y_dot + (1 - alfa) * x_dot(1,0);
    double  Fi_dot = alfa * Fi_dot + (1 - alfa) * x_dot(2,0);
    double X = x_k(0,0);
    double Y = x_k(1,0);
    double Fi = x_k(2,0);
    odomNew.pose.pose.position.x = X;
    odomNew.pose.pose.position.y = Y;
    odomNew.pose.pose.position.z = 0.0;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Fi);
    odomNew.pose.pose.orientation = odom_quat;
    if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) || isnan(odomNew.pose.pose.position.z))  
    {
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
        odomNew.pose.pose.orientation = odomOld.pose.pose.orientation;
    }
    for(int i = 0; i<36; i++) 
    {
        if(i == 0 || i == 7 || i == 14) 
        {
        odomNew.pose.covariance[i] = .01;
        }
        else if (i == 21 || i == 28 || i== 35) {
        odomNew.pose.covariance[i] += 0.1;
        }
        else 
        {
        odomNew.pose.covariance[i] = 0;
        }
    }
    odomPose2D.x = odomNew.pose.pose.position.x;
    odomPose2D.y = odomNew.pose.pose.position.y;
    tf::Quaternion q(
        odomNew.pose.pose.orientation.x,
        odomNew.pose.pose.orientation.y,
        odomNew.pose.pose.orientation.z,
        odomNew.pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    odomPose2D.theta = yaw;
    odomPose2D_pub.publish(odomPose2D);
    if (odomPose2D.theta > PI) 
    {
        odomPose2D.theta -= 2 * PI;
    }
    else if (odomPose2D.theta < -PI) 
    {
        odomPose2D.theta += 2 * PI;
    }
    else
    {

    }
    ROS_DEBUG_STREAM("X = odomPose2D.x="<<odomPose2D.x);
    ROS_DEBUG_STREAM("Y = odomPose2D.x="<<odomPose2D.y);
    ROS_DEBUG_STREAM("Fi = odomPose2D.x="<<odomPose2D.theta);
    odomNew.twist.twist.linear.x = X_dot;  
    odomNew.twist.twist.linear.y = Y_dot;
    odomNew.twist.twist.angular.z = Fi_dot;  
    ROS_DEBUG_STREAM("V = odomNew.twist.twist.linear.x ="<<odomNew.twist.twist.linear.x);
    ROS_DEBUG_STREAM("V = odomNew.twist.twist.linear.y ="<<odomNew.twist.twist.linear.y);
    ROS_DEBUG_STREAM("Fi_dot = odomNew.twist.twist.angular.z ="<<odomNew.twist.twist.angular.z);
    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
    odomOld.pose.pose.orientation = odomNew.pose.pose.orientation;
    odomOld.header.stamp = odomNew.header.stamp;
    odom_data_pub.publish(odomNew);
}

int main(int argc, char **argv) 
{
    ros::init(argc, argv, "odom_pub1");
    ros::NodeHandle node;  
    odomNew.header.frame_id = "odom";
    odomNew.child_frame_id = "base_footprint";
    odomNew.pose.pose.position.z = 0;
    odomNew.pose.pose.orientation.x = 0;
    odomNew.pose.pose.orientation.y = 0;
    odomNew.twist.twist.linear.x = 0;
    odomNew.twist.twist.linear.y = 0;
    odomNew.twist.twist.linear.z = 0;
    odomNew.twist.twist.angular.x = 0;
    odomNew.twist.twist.angular.y = 0;
    odomNew.twist.twist.angular.z = 0;
    odomOld.pose.pose.position.x = initialX;
    odomOld.pose.pose.position.y = initialY;
    odomOld.pose.pose.orientation.z = initialFi;
    x_k	<<	initialX,	initialY,	initialFi;
    x_k1	<<	initialX,	initialY,	initialFi;
    x_dot	<<	0.0,	0.0,	0.0;
    A	<< 	1.0,	0,		0,
            0,		1.0,	0,
            0,		0,		1.0; 
    H	<<	1.0,	0,		0,
            0,		1.0,	0,
            0,		0,		1.0;
            
    P_k	<<	100.1,	0,		  0,
            0,		  100.1,	0,
            0,		  0,		  100.1;
            
    R_k	<<	0.1,	0,		0,
            0,		0.1,	0,
            0,		0,		0.1;  
    Q_k	<<	0.1,	0,		0,
            0,		0.1,	0,
            0,		0,		0.1;
    ros::Subscriber subInitialPose = node.subscribe("initial_2d", 1, set_initial_2d);
    ros::Subscriber subLidarLocPose = node.subscribe("localizationcontroller/out/localizationcontroller_result_message_0502", 1, LidarLocPose);
    ros::Subscriber subForTickCounts = node.subscribe("/amr/base_controller/servo_vel_kola", 1, GetVw,  ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForAngeCounts = node.subscribe("/amr/base_controller/servo_Psi_kola", 1, GetPsi, ros::TransportHints().tcpNoDelay());  
    odom_data_pub = node.advertise<nav_msgs::Odometry>("amr/odom_pub/odom", 1);
    odom_data_pub_quat = node.advertise<nav_msgs::Odometry>("amr/odom_pub/odom_data_quat", 1);
    sickLidarPose2D_pub = node.advertise<geometry_msgs::Pose2D>("amr/odom_pub/sickLidarPose2D", 1);
    odomPose2D_pub =  node.advertise<geometry_msgs::Pose2D>("amr/odom_pub/odomPose2D", 1);
    ros::Rate loop_rate(10);   
    while(ros::ok()) 
    {
        ROS_DEBUG("LOOP");
        update_odom();
        publish_quat();
        update_trans();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

