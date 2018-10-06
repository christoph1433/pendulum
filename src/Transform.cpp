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
ros::Subscriber input_roll;
ros::Subscriber input_pitch;
ros::Subscriber groundtruth_orient_subscriber;
ros::Publisher output_roll;
ros::Publisher output_pitch;

// Global Variables
//body coordinates
float roll, pitch, yaw;
//current and transformed control inputs
float u_y = 0;
float u_x = 0;
float new_u_y = 0;
float new_u_x = 0;
//General
double time_stamp;

//Functions
void MsgCallback1(const std_msgs::Float32);
void MsgCallback2(const std_msgs::Float32);
void MsgCallback3(const geometry_msgs::PoseStamped& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //subscriber and publisher topics
  input_roll = node.subscribe("/swing_up/y_input", 1000, MsgCallback1);
  input_pitch = node.subscribe("/swing_up/x_input", 1000, MsgCallback2);
  groundtruth_orient_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback3);
  output_roll = node.advertise<std_msgs::Float32>("/swing_up/y_input_transformed", 1000);
  output_pitch = node.advertise<std_msgs::Float32>("/swing_up/x_input_transformed", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Transform.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      {
        time_stamp = ros::Time::now().toSec();                

        //convert world to body frame inputs
        new_u_y = u_y*cos(yaw)+u_x*sin(-yaw);
        new_u_x = u_x*cos(-yaw)+u_y*sin(yaw);
        
        //saturate input
        if(u_y > 0.7 || u_y != u_y){
          u_y = 0.7;  
        }
        else if (u_y < -0.7){
          u_y = -0.7;
        }

        if(u_x > 0.7 || u_x != u_x){
          u_x = 0.7;  
        }
        else if (u_x < -0.7){
          u_x = -0.7;
        }

        //Debug output
        // ROS_INFO("u_y: %f",u_y);
        // ROS_INFO("u_x: %f",u_x);
        // ROS_INFO("yaw: %f",yaw);
        // ROS_INFO("new_u_y: %f",new_u_y);
        // ROS_INFO("new_u_x: %f",new_u_x);

        //Publish
        std_msgs::Float32 u;
        u.data = new_u_y;
        output_roll.publish(u);
        
        std_msgs::Float32 r;
        r.data = new_u_x;
        output_pitch.publish(r);

        ros::spinOnce();
        rate.sleep();

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_y << "\t" << u_x << "\t" << yaw << "\t" << new_u_y << "\t" << new_u_x << endl;

      }
  myfile.close();
  return 0;
};

//subscribe to control input for position y
void MsgCallback1(const std_msgs::Float32 msg)
{
  u_y = msg.data;
}

//subscribe to control input for position x
void MsgCallback2(const std_msgs::Float32 msg)
{
  u_x = msg.data;
}

//quad rotation
void MsgCallback3(const geometry_msgs::PoseStamped& msg)
{
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    yaw = -yaw;
}