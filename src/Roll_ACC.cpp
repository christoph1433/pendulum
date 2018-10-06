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
ros::Subscriber input_subscriber;
ros::Publisher control_publisher;

//Global Variables
//PID gains
float KP_Roll = 300; 
float KI_Roll = 0.8; 
float KD_Roll = 43;
//PID variables
float error_prior_roll = 0;
float error_roll = 0;
float integral_roll = 0;
float derivative_roll = 0;
//control input
float u_roll = 0; 
//setup timestep
float iteration_time = 0.001;
//des roll acceleration from LQR
float des_roll = 0;   
//Ground Truth variables
float roll, pitch, yaw;
//General
int flag = 0;
double time_stamp;

//Functions
void MsgCallback(const geometry_msgs::PoseStamped& msg);
void MsgCallback2(const std_msgs::Float32);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //subscriber and publisher topics
  groundtruth_orient_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback);
  input_subscriber = node.subscribe("/swing_up/y_input_acc", 1000, MsgCallback2);
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/roll_input_acc", 1000);

  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_Roll.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      {
        time_stamp = ros::Time::now().toSec();     

        //PID loop for roll acceleration
        error_roll = des_roll - roll;
        if(flag == 1){
          integral_roll = integral_roll + (error_roll*iteration_time);
          derivative_roll = (error_roll - error_prior_roll)/iteration_time;

          if(integral_roll!=integral_roll  || integral_roll-integral_roll != 0){integral_roll=0;}
        }
        u_roll = (KP_Roll * error_roll + KI_Roll * integral_roll + KD_Roll * derivative_roll);
        error_prior_roll = error_roll;

        //saturate input
        if(u_roll > 200 || u_roll != u_roll){
          //ROS_INFO("Thrust is: %f",u_roll);
          u_roll = 200;  
        }
        else if (u_roll < -200){
          u_roll = -200;
        }

        //Debug output
        // ROS_INFO("actual_roll: %f",roll);
        // ROS_INFO("des_roll: %f",des_roll);
        // ROS_INFO("error_roll: %f",error_roll);
        // ROS_INFO("integral_roll: %f",integral_roll);
        // ROS_INFO("derivative_roll: %f",derivative_roll);
        // ROS_INFO("error_prior_roll: %f",error_prior_roll);
        // ROS_INFO("u_roll: %f",u_roll);

        //Publish
        std_msgs::Float32 u;
        u.data = u_roll;

        control_publisher.publish(u);
        ros::spinOnce();
        rate.sleep();

        flag += 1;
        if(flag>1){flag = 1;}

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_roll << "\t" << roll << "\t" << des_roll <<endl;


      }
  myfile.close();
  return 0;
};

//quad rotation
void MsgCallback(const geometry_msgs::PoseStamped& msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}

//des roll acceleration from LQR lateral
void MsgCallback2(const std_msgs::Float32 msg)
{
  des_roll = msg.data;
}