#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "math.h"
#include "tf/transform_datatypes.h"
#include "tf/LinearMath/Matrix3x3.h"
#include <mav_msgs/Actuators.h>
#include <ros/console.h>
#include <time.h>
#include <fstream>
#include <iostream>

using namespace std;

//Global Publisher and Subscriber
ros::Subscriber angle_pendulum;
ros::Subscriber angular_velocity_pendulum;
ros::Publisher output_des_linear_acceleration;

// Global Variables
//Ground Truth pendulum
float roll, pitch, yaw;
float vel_roll, vel_pitch, vel_yaw;
//control input
float u_y = 1;
//Energy controller
float mp = 0.025670828; // Mass of the pendulum in kg
float lp = 0.35/2; // half of the length of pendulum in m
float Ip = 0.0007862; //Inertia
float k = 19.5; // control input multiplier
float g = 9.8; // Gravity
float n = 10; //n*g is saturaion for control input
float energy;
int sign;
//General
double begin_time = 0;
double duration = 0;
double time_stamp;
int flag = 0;

//Functions
void MsgCallback1(const geometry_msgs::PoseStamped& msg);
void MsgCallback2(const sensor_msgs::Imu& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //subscriber and publisher topics
  angle_pendulum = node.subscribe("/hummingbird_pendulum_pend/ground_truth/pose", 1000, MsgCallback1);
  angular_velocity_pendulum = node.subscribe("/hummingbird_pendulum_pend/ground_truth/imu", 1000, MsgCallback2);
  output_des_linear_acceleration = node.advertise<std_msgs::Float32>("/swing_up/des_y_acc", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/EnergieControlLateral.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      {
        if(flag == 0){begin_time = ros::Time::now().toSec(); }
        time_stamp = ros::Time::now().toSec();                
        duration = time_stamp - begin_time;

        //Energy controller
        if(duration < 0.5){
          u_y = 0;  
        }
        else {
          energy = 0.5*Ip*vel_roll*vel_roll+(mp*g*lp*(1-cos(roll)));

          if(roll==M_PI){
            energy = 0;
          }
          else if(roll<0.1){
            energy = mp*g*lp;
          }

          if(vel_roll*cos(roll) < 0){
            sign = -1;
          }
          else if(vel_roll*cos(roll) > 0){
            sign = 1;
          }
          else{
            sign = 1;
          }

          if(k*energy*sign > n*g){
            u_y = n*g;
          }
          else if(k*energy*sign < -n*g){
            u_y=-n*g;
          }
          else{
            u_y = k*energy*sign;
          }
        }            

        // DEBUG
        // ROS_INFO("roll: %f",roll);
        // ROS_INFO("vel_roll: %f",vel_roll);
        // ROS_INFO("u_y: %f",u_y);

        //Publish
        std_msgs::Float32 u;
        u.data = u_y;
        output_des_linear_acceleration.publish(u);

        ros::spinOnce();
        rate.sleep();

        //write into file
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_y << "\t" << roll << "\t" << vel_roll << endl;

        flag += 1;
        if(flag>3){flag = 3;}
      }
  myfile.close();
  return 0;
};

//pendulum pose
void MsgCallback1(const geometry_msgs::PoseStamped& msg)
{  
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // roll = roll + pi;
}

//pendulum velocity
void MsgCallback2(const sensor_msgs::Imu& msg)
{
  vel_roll = -msg.angular_velocity.x;
}