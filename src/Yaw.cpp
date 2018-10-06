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
ros::Subscriber groundtruth_orient_subscriber;
ros::Publisher control_publisher;

// Global Variables
//PID gains
float KP_Yaw = 600; 
float KI_Yaw = 0; 
float KD_Yaw = 180;
//PID variables
float error_prior_yaw = 0;
float error_yaw = 0;
float integral_yaw = 0;
float derivative_yaw = 0;
//control input
float u_yaw = 0;
//quad rot
float roll, pitch, yaw;
//des yaw
float des_yaw = 0;
//General
int flag = 0;
double time_stamp;
double iteration_time = 0.001;

//Functions
void MsgCallback(const geometry_msgs::PoseStamped& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //subscriber and publisher topics
  groundtruth_orient_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback);
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/yaw_input", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_Yaw.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      { 
        time_stamp = ros::Time::now().toSec();                           

        //PID loop for aw
        error_yaw = des_yaw - yaw;
        if(flag == 1 || flag == 2 || flag == 3){
          integral_yaw = integral_yaw + (error_yaw*iteration_time);
          derivative_yaw = (error_yaw - error_prior_yaw)/iteration_time;

          if(integral_yaw!=integral_yaw  || integral_yaw-integral_yaw != 0){integral_yaw=0;}
        }
        u_yaw = (KP_Yaw * error_yaw + KI_Yaw * integral_yaw + KD_Yaw * derivative_yaw);
        error_prior_yaw = error_yaw;

        //saturate input
        if(u_yaw > 350 || u_yaw != u_yaw){
        //  ROS_INFO("Yaw is: %f",u_yaw);
          u_yaw = 350;  
        }
        else if (u_yaw < -350){
          u_yaw = -350;
        }

        //Debug output
        // ROS_INFO("actual_yaw: %f",yaw);
        // ROS_INFO("des_yaw: %f",des_yaw);
        // ROS_INFO("error_yaw: %f",error_yaw);
        // ROS_INFO("integral_yaw: %f",integral_yaw);
        // ROS_INFO("derivative_yaw: %f",derivative_yaw);
        // ROS_INFO("error_prior_yaw: %f",error_prior_yaw);
        // ROS_INFO("u_yaw: %f",u_yaw);

        //Publish
        std_msgs::Float32 u;
        u.data = u_yaw;

        control_publisher.publish(u);
        ros::spinOnce();
        rate.sleep();

        flag += 1;
        if(flag>3){flag = 3;}

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_yaw << "\t" << yaw << "\t" << des_yaw << endl;


      }
  myfile.close();
  return 0;
};

// quad orientation
void MsgCallback(const geometry_msgs::PoseStamped& msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    //roll = - roll;
}