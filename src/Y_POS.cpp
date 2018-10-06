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
float KP_Y = 0.25; 
float KI_Y = 0;
float KD_Y = 0.2;
//PID variables
float error_prior_y = 0;
float error_y = 0;
float integral_y = 0;
float derivative_y = 0;
//control input
float u_y = 0;
//quad pose
float pos_y;
//desired position y
float des_y = 0;
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
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/y_input", 1000);

  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_Y.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      { 
        //start timer 
        if(flag == 0){begin_time = ros::Time::now().toSec(); }
        time_stamp = ros::Time::now().toSec();                
        duration = time_stamp - begin_time;

        if(duration < 15 ){
          des_y = 0;  
        }
        //bring quad to desired position if wanted
        else {
         des_y = 0;
        } 

        //PID loop for position y
        error_y = des_y - pos_y;
        if(flag == 1){
          integral_y = integral_y + (error_y*iteration_time);
          derivative_y = (error_y - error_prior_y)/iteration_time;

          if(integral_y!=integral_y  || integral_y-integral_y != 0){integral_y=0;}
        }
        u_y = (KP_Y * error_y + KI_Y * integral_y + KD_Y * derivative_y);
        error_prior_y = error_y;

        //saturate input
        if(u_y > 0.7 || u_y != u_y){
          u_y = 0.7;  
        }
        else if (u_y < -0.7){
          u_y = -0.7;
        }

        //Debug output
        // ROS_INFO("actual_y: %f",pos_y);
        // ROS_INFO("des_y: %f",des_y);
        // ROS_INFO("error_y: %f",error_y);
        // ROS_INFO("integral_y: %f",integral_y);
        // ROS_INFO("derivative_y: %f",derivative_y);
        // ROS_INFO("error_prior_y: %f",error_prior_y);
        // ROS_INFO("u_y: %f",u_y);

        //Publish
        std_msgs::Float32 u;
        u.data = -u_y;

        control_publisher.publish(u);
        ros::spinOnce();
        rate.sleep();

        flag += 1;
        if(flag>1){flag = 1;}

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_y << "\t" << pos_y << "\t" << des_y << endl;
      }
  myfile.close();
  return 0;
};

//quad pose y
void MsgCallback(const geometry_msgs::PoseStamped& msg)
{
    pos_y = msg.pose.position.y;
}
