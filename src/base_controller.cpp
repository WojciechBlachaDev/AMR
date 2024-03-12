#include "ros/ros.h"
#include <string>
#include "std_msgs/Int64.h"
#include "std_msgs/Float32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <cmath>

ros::Publisher comand_curtis_vel_pub;
ros::Publisher comand_servo_angle_pub;
ros::Publisher servo_vel_pub;
ros::Publisher servo_vel_kola_pub;
ros::Publisher servo_Psi_kola_pub;
ros::Publisher V_cel_pub;
ros::Time current_time, last_time;
nav_msgs::Odometry Odom;
std_msgs::Int64 PWM_DutyCycle;
std_msgs::Float32 ServoTick;
std_msgs::Float32 V_wozka;
std_msgs::Float32 V_kola;
std_msgs::Float32 Psi_kola;
std_msgs::Float32 V_cel_msg;
tf::TransformBroadcaster *tb;
const double PI = 3.141592;
const double BASE = 1.195;
const double ZERO_SERVO = 0.0;
const double RADIN_MIN = (-42 * PI)/180.0;
const double RADIN_MAX = (42 * PI)/180.0;
double DeltaDistanceKolo = 0.0;
double DeltaDistanceKolo_1 = 0.0;
double Psi = 0.0;
double V = 0;
float Vw = 0;
double Fi = 0;
double Fi_dot = 0;
double controlWozekPWM  = 0;
double controlKoloAngle  = 0;
double V_zad = 0;
double V_cel = 0;
double X_dot1 = 0;
double Y_dot1 = 0;
double Fi_dot_zad = 0;
double Fi_dot1 = 0;
double d_t = 0;
float Vw1 = 0;
using namespace std;

void DistanceCalc(const std_msgs::Float32& Count) 
{
    ROS_DEBUG_STREAM("DistanceCalc");
    double WozekVw = Count.data;
    float alfa = 0.0;
    Vw = WozekVw;
    ROS_DEBUG_STREAM("Wozek_speed = "<<Count.data);
}

void AngleCalc(const std_msgs::Float32& Count) 
{
    Psi = (PI / 180) * (double)Count.data;
    ROS_DEBUG_STREAM("AngleCalc Psi Count.data = "<<Count.data<<" Psi = "<<Psi);
}

void OdomData(const nav_msgs::Odometry OdomGet)  
{
    X_dot1 = (double)OdomGet.twist.twist.linear.x;
    Y_dot1 = (double)OdomGet.twist.twist.linear.y;
    Fi_dot1 = (double)OdomGet.twist.twist.angular.z;
	tf::Quaternion q(
    OdomGet.pose.pose.orientation.x,
    OdomGet.pose.pose.orientation.y,
    OdomGet.pose.pose.orientation.z,
    OdomGet.pose.pose.orientation.w);
    tf::Matrix3x3 m(q); 
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    Fi = yaw;
    Fi_dot = Fi_dot1;
    V = cos(Fi) * X_dot1 + sin(Fi) * Y_dot1;
    V_cel = V;
}

void VelDest(const geometry_msgs::Twist& cmdVel) 
{
    double V_zad_1 = (double)cmdVel.linear.x;
    double Fi_dot_zad_1 = (double)cmdVel.angular.z;
    float alfa = 0.0;
    V_zad = alfa * V_zad + (1 - alfa) * V_zad_1;
    Fi_dot_zad = alfa * Fi_dot_zad + (1 - alfa) * Fi_dot_zad_1;

}

void BaseControler()  
{          
    ROS_DEBUG("BaseControler");
    double Delta_V = (V_zad - V);
    double regulator = 0;
    if (abs(V_zad) != 0.0)
    {
        regulator = 1000; 
    }
    else 
    {
        regulator = 1500;
    }
    controlWozekPWM = controlWozekPWM + (regulator * Delta_V);
    if ((controlWozekPWM > -1200) && (controlWozekPWM < 1200))
    {
        controlWozekPWM = controlWozekPWM;
    }   
    else
    {
        if (controlWozekPWM>=1200)
        {
        controlWozekPWM=1199;
        }
        else if (controlWozekPWM<=-1200)
        {
        controlWozekPWM= -1199;
        }
    }
    ROS_DEBUG_STREAM("BaseControler V_zad ="<<V_zad<<" V="<<V<<" Delta_V="<<Delta_V<<" controlWozekPWM ="<<controlWozekPWM);
    if (Vw!= 0) 
    {
        double arg = (Fi_dot_zad * BASE)/ Vw ;
        if (arg > 1) 
        {
        arg = 1;
        }
        else if (arg < -1)  
        {
        arg = -1;
        }
        else
        {
        arg = arg;
        }
        controlKoloAngle = (180.0 / PI) * asin(arg);
    }
    else  
    {
        controlKoloAngle = 0;
    }
    if ((controlKoloAngle <= 90) && (controlKoloAngle >= -90))  
    {
        controlKoloAngle = controlKoloAngle;
    }
    else  
    {
        if (controlKoloAngle > 90)  
        {  
        controlKoloAngle = 90;
        }
        else if (controlKoloAngle < -90)  
        {  
        controlKoloAngle = -90;
        }
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "base_controler");
    ros::NodeHandle node; 
    current_time = ros::Time::now();
    last_time = ros::Time::now();  
    ros::Subscriber subForTickCounts = node.subscribe("amr/base_controller/data/speed", 1, DistanceCalc,  ros::TransportHints().tcpNoDelay());
    ros::Subscriber subForAngeCounts = node.subscribe("amr/base_controller/data/angle", 1, AngleCalc, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subCmdVel = node.subscribe("cmd_vel", 1, VelDest, ros::TransportHints().tcpNoDelay());
    ros::Subscriber subOdom = node.subscribe("amr/odom_pub/odom", 1, OdomData, ros::TransportHints().tcpNoDelay());
    comand_curtis_vel_pub = node.advertise<std_msgs::Int64>("amr/steering/curtis", 1);
    comand_servo_angle_pub = node.advertise<std_msgs::Float32>("amr/steering/servo", 1);
    servo_vel_pub = node.advertise<std_msgs::Float32>("/amr/base_controller/servo_vel", 1);
    servo_vel_kola_pub = node.advertise<std_msgs::Float32>("/amr/base_controller/servo_vel_kola", 1);
    servo_Psi_kola_pub = node.advertise<std_msgs::Float32>("/amr/base_controller/servo_Psi_kola", 1);
    V_cel_pub = node.advertise<std_msgs::Float32>("/amr/base_controller/V_cel", 1);
    ros::Rate loop_rate(10); 
    while(ros::ok()) 
    {
        ROS_DEBUG("LOOP CONTROLER");
        BaseControler();
        PWM_DutyCycle.data = controlWozekPWM;
        comand_curtis_vel_pub.publish((std_msgs::Int64)PWM_DutyCycle);
        ServoTick.data  = controlKoloAngle;
        comand_servo_angle_pub.publish((std_msgs::Float32)ServoTick);
        V_wozka.data = V;
        servo_vel_pub.publish((std_msgs::Float32)V_wozka);      
        V_kola.data = Vw;
        servo_vel_kola_pub.publish((std_msgs::Float32)V_kola); 
        Psi_kola.data = Psi;
        servo_Psi_kola_pub.publish((std_msgs::Float32)Psi_kola); 
        V_cel_msg.data = V_cel;
        V_cel_pub.publish((std_msgs::Float32)V_cel_msg); 
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

