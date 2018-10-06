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
float KP_Pitch = 300; 
float KI_Pitch = 11; 
float KD_Pitch = 43;
//PID variables
float error_prior_pitch = 0;
float error_pitch = 0;
float integral_pitch = 0;
float derivative_pitch = 0;
//control input
float u_pitch = 0;
//Ground Truth variables
float roll, pitch, yaw;
//des pitch
float des_pitch = 0; 
//General
int flag = 0;
double time_stamp;
double iteration_time = 0.001;

//Functions
void MsgCallback(const geometry_msgs::PoseStamped& msg);
void MsgCallback2(const std_msgs::Float32);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;
  
  //subscriber and publisher topics
  groundtruth_orient_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback);
  input_subscriber = node.subscribe("/swing_up/x_input_transformed", 1000, MsgCallback2);
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/pitch_input", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_Pitch.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      {
        time_stamp = ros::Time::now().toSec();                

        //PID loop for Pitch
        error_pitch = des_pitch - pitch;
        if(flag == 1){
          integral_pitch = integral_pitch + (error_pitch*iteration_time);
          derivative_pitch = (error_pitch - error_prior_pitch)/iteration_time;

          if(integral_pitch!=integral_pitch  || integral_pitch-integral_pitch != 0){integral_pitch=0;}
        }
        u_pitch = (KP_Pitch * error_pitch + KI_Pitch * integral_pitch + KD_Pitch * derivative_pitch);
        error_prior_pitch = error_pitch;

        //saturate input
        if(u_pitch > 200 || u_pitch != u_pitch){
          u_pitch = 200;  
        }
        else if (u_pitch < -200){
          u_pitch = -200;
        }

        //Debug output
        // ROS_INFO("actual_pitch: %f",pitch);
        // ROS_INFO("des_pitch: %f",des_pitch);
        // ROS_INFO("error_pitch: %f",error_pitch);
        // ROS_INFO("integral_pitch: %f",integral_pitch);
        // ROS_INFO("derivative_pitch: %f",derivative_pitch);
        // ROS_INFO("error_prior_pitch: %f",error_prior_pitch);
        // ROS_INFO("u_pitch: %f",u_pitch);

        //Publish
        std_msgs::Float32 u;
        u.data = u_pitch;

        control_publisher.publish(u);
        ros::spinOnce();
        rate.sleep();

        flag += 1;
        if(flag>1){flag = 1;}

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_pitch << "\t" << pitch << "\t" << des_pitch << endl;


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
    //roll = - roll;
}

//des pitch from translation
void MsgCallback2(const std_msgs::Float32 msg)
{
  des_pitch = msg.data;
}