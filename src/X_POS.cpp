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
ros::Subscriber groundtruth_pos_subscriber;
ros::Publisher control_publisher;

// Global Variables
//PID gains
float KP_X = 0.25;
float KI_X = 0.12;
float KD_X = 0.2;
//PID variables
float error_prior_x = 0;
float error_x = 0;
float integral_x = 0;
float derivative_x = 0;
//control input
float u_x = 0;
//quad pose
float pos_x;
//des pos
float des_x = 0;
//General
int flag = 0;
double begin_time = 0;
double duration = 0;
double time_stamp;
double iteration_time = 0.001;

//Functions
void MsgCallback(const geometry_msgs::PoseStamped& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //subscriber and publisher topics
  groundtruth_pos_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback);
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/x_input", 1000);

  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_X.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      {
        //start timer 
        if(flag == 0){begin_time = ros::Time::now().toSec(); }
        time_stamp = ros::Time::now().toSec();                
        duration = time_stamp - begin_time;

        if(duration < 10 ){
          des_x = 0;  
        }
        //bring quad to desired position if wanted
        else {
         des_x = 0;
        }           
              
        //PID loop for position x
        error_x = des_x - pos_x;
        if(flag == 1){
          integral_x = integral_x + (error_x*iteration_time);
          derivative_x = (error_x - error_prior_x)/iteration_time;

          if(integral_x!=integral_x  || integral_x-integral_x != 0){integral_x=0;}
        }
        u_x = (KP_X * error_x + KI_X * integral_x + KD_X * derivative_x);
        error_prior_x = error_x;

        //saturate input
        if(u_x > 0.5 || u_x != u_x){
          u_x = 0.5;  
        }
        else if (u_x < -0.5){
          u_x = -0.5;
        }

        //Debug output
        // ROS_INFO("actual_x: %f",pos_x);
        // ROS_INFO("des_x: %f",des_x);
        // ROS_INFO("error_x: %f",error_x);
        // ROS_INFO("integral_x: %f",integral_x);
        // ROS_INFO("derivative_x: %f",derivative_x);
        // ROS_INFO("error_prior_x: %f",error_prior_x);
        // ROS_INFO("u_x: %f",u_x);

        //Publish
        std_msgs::Float32 u;
        u.data = u_x;

        control_publisher.publish(u);
        ros::spinOnce();
        rate.sleep();

        flag += 1;
        if(flag>1){flag = 1;}

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_x << "\t" << pos_x << "\t" << des_x << endl;
      }
  myfile.close();
  return 0;
};

//quad pose x
void MsgCallback(const geometry_msgs::PoseStamped& msg)
{
    pos_x = msg.pose.position.x;
}
