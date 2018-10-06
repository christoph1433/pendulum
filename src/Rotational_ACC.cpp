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
ros::Subscriber groundtruth_acc_subscriber;
ros::Subscriber des_yaw_acc_subscriber;
ros::Publisher control_publisher;

// Global Variables
//PID gains
float KP_Yaw = 10;
float KI_Yaw = 200/4; 
float KD_Yaw = 1;
//PID variables
float error_prior_yaw_acc = 0;
float error_yaw_acc = 0;
float integral_yaw_acc = 0;
float derivative_yaw_acc = 0;
//control input
float u_yaw_acc = 0;
//desired rotational acc
float des_yaw_acc = 0;    
//General
int flag = 0;
double iteration_time = 0.001;
double time_stamp;
//quad body rotational values
float roll_acc, pitch_acc, yaw_acc;
float vel_yaw_old, vel_yaw; 
//average filter
int sum_flag = 0;
float sum_yaw_accel = 0;
float ave_yaw_accel= 0;
float accelerations[250] = { };
int c = 0;
int accel_count = 0;

//Functions
void MsgCallback(const sensor_msgs::Imu& msg);
void MsgCallback2(const std_msgs::Float32);
void accel_pub(void);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //subscriber and publisher topics
  groundtruth_acc_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/imu", 1000, MsgCallback);
  des_yaw_acc_subscriber = node.subscribe("/swing_up/des_yaw_acc", 1000, MsgCallback2);
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/yaw_input_acc", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_Yaw_Acc.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      { 
         time_stamp = ros::Time::now().toSec();                           

        //PID loop for acceleration yaw
        error_yaw_acc = des_yaw_acc - ave_yaw_accel;
        if(flag == 1){
          integral_yaw_acc = integral_yaw_acc + (error_yaw_acc*iteration_time);
          derivative_yaw_acc = (error_yaw_acc - error_prior_yaw_acc)/iteration_time;

          if(integral_yaw_acc!=integral_yaw_acc  || integral_yaw_acc-integral_yaw_acc != 0){integral_yaw_acc=0;}
        }
        u_yaw_acc = (KP_Yaw * error_yaw_acc + KI_Yaw * integral_yaw_acc + KD_Yaw * derivative_yaw_acc);
        error_prior_yaw_acc = error_yaw_acc;

        if(u_yaw_acc > 300 || u_yaw_acc != u_yaw_acc){
        //  ROS_INFO("Yaw is: %f",u_yaw);
          u_yaw_acc = 300;  
        }
        else if (u_yaw_acc < -300){
          u_yaw_acc = -300;
        }

        //Debug output
        // ROS_INFO("actual_yaw_acc: %f",ave_yaw_accel);
        // ROS_INFO("des_yaw_acc: %f",des_yaw_acc);
        // ROS_INFO("error_yaw_acc: %f",error_yaw_acc);
        // ROS_INFO("integral_yaw_acc: %f",integral_yaw_acc);
        // ROS_INFO("derivative_yaw_acc: %f",derivative_yaw_acc);
        // ROS_INFO("error_prior_yaw_acc: %f",error_prior_yaw_acc);
        // ROS_INFO("u_yaw_acc: %f",u_yaw_acc);

        //Publish
        std_msgs::Float32 u;
        u.data = u_yaw_acc;

        control_publisher.publish(u);
        ros::spinOnce();
        rate.sleep();

        flag += 1;
        if(flag>1){flag = 1;}

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_yaw_acc << "\t" << ave_yaw_accel << "\t" << des_yaw_acc << endl;


      }
  myfile.close();
  return 0;
};

//get rotational velocity, calculate accel and filter it
void MsgCallback(const sensor_msgs::Imu& msg)
{ 
  vel_yaw_old = vel_yaw;    
  vel_yaw = msg.angular_velocity.z;
  yaw_acc = (vel_yaw - vel_yaw_old) * 1000;
 // ROS_INFO("yaw_acc: %f",yaw_acc);

  accelerations[sum_flag] = yaw_acc;
  sum_yaw_accel += yaw_acc;
  if(accel_count == 249){
	  ave_yaw_accel = sum_yaw_accel/250;
	  sum_yaw_accel = sum_yaw_accel - accelerations[c];

	  sum_flag = c;
	  c++;
	  if(c>249){
	  	c=0;
	  }
  }

  if(accel_count<249){
    sum_flag++;
  	accel_count++;
  }
}

//des acceleration in yaw from rotational energy controller
void MsgCallback2(const std_msgs::Float32 msg)
{
	des_yaw_acc = msg.data;
}