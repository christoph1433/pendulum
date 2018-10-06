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
ros::Subscriber truth_pose_subscriber;
ros::Publisher control_publisher;

// Global Variables
//PID gains
float KP_Thrust = 400; 
float KI_Thrust = 5.0; 
float KD_Thrust =175;
//PID variables
float error_prior_z = 0;
float error_z = 0;
float integral_z = 0;
float derivative_z = 0;
// control input
float u_thrust = 0;
//actual pos
float pos_z = 0;
//setup timestepc
float iteration_time = 0.001;
//des altitude
float des_pos_z = 1;
//General
int flag = 0;
double time_stamp;

//Functions
void MsgCallback(const geometry_msgs::PoseStamped& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //subscriber and publisher topics
  truth_pose_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback);
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/thrust_input", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_Thrust.txt", ios::trunc);

  ros::Rate rate(1000);
  	while (node.ok())
  		{
        time_stamp = ros::Time::now().toSec();           			
        
  			//PID loop for position z
  			error_z = des_pos_z - pos_z;
        if(flag == 1){
    			integral_z = integral_z + (error_z*iteration_time);
    			derivative_z = (error_z - error_prior_z)/iteration_time;
          
          if(integral_z!=integral_z  || integral_z-integral_z != 0){integral_z=0;}

        }
  			u_thrust = (KP_Thrust * error_z + KI_Thrust * integral_z + KD_Thrust * derivative_z);
  			error_prior_z = error_z;

        if(u_thrust > 500 || u_thrust != u_thrust){
          //ROS_INFO("Thrust is: %f",u_thrust);
          u_thrust = 500;  
        }
        else if (u_thrust < -300){
          u_thrust = -300;
        }

        //Debug output
        // ROS_INFO("actual_z: %f",pos_z);
        // ROS_INFO("des_z: %f",des_pos_z);
        // ROS_INFO("error_z: %f",error_z);
        // ROS_INFO("integral_z: %f",integral_z);
        // ROS_INFO("derivative_z: %f",derivative_z);
        // ROS_INFO("error_prior_z: %f",error_prior_z);
        // ROS_INFO("u_thrust: %f",u_thrust);

  			//Publish
  			std_msgs::Float32 u;
  			u.data = u_thrust;

  			control_publisher.publish(u);
        ros::spinOnce();
    		rate.sleep();

        flag += 1;
        if(flag>1){flag = 1;}

        //write into file to analyze data
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_thrust << "\t" << pos_z << "\t" << des_pos_z << endl;


  		}
  myfile.close();
  return 0;
};

//altitude of quad
void MsgCallback(const geometry_msgs::PoseStamped& msg)
{
    pos_z = msg.pose.position.z;
}