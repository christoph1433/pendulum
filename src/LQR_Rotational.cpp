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
ros::Subscriber pos_quad;
ros::Subscriber angular_velocity_quad;
ros::Publisher output_roll;

// Global Variables
//pendulum states
float pend_roll, pend_pitch, pend_yaw;
float pend_vel_roll;
//quadrotor states
float quad_roll, quad_pitch, quad_yaw;
float quad_vel_yaw;
//error values to get to desired states
float error_pend_roll, error_pend_vel_roll, error_quad_yaw, error_quad_vel_yaw;
//desired states
float des_pend_roll = 0;
float des_pend_vel_roll = 0;
float des_quad_yaw = 0;
float des_quad_vel_yaw = 0;
//lqr gains calculated using MATLAB
float K_quad_yaw = 0.2236;
float K_quad_vel_yaw = 1.3040;
float K_pend_roll = 13.4454;
float K_pend_vel_roll = 2.3025;
//control input
float u_yaw = 0;
//timestamp
double time_stamp;
//system characteristics
float l = 0.17; //arm length of quad
float k = 0.00000854858; //thrust constant of motor

//callback functions
void MsgCallback1(const geometry_msgs::PoseStamped& msg);
void MsgCallback2(const sensor_msgs::Imu& msg);
void MsgCallback3(const geometry_msgs::PoseStamped& msg);
void MsgCallback4(const sensor_msgs::Imu& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //defining arguments for subscribing and publishing to topics
  angle_pendulum = node.subscribe("/hummingbird_pendulum_pend/ground_truth/pose", 1000, MsgCallback1);
  angular_velocity_pendulum = node.subscribe("/hummingbird_pendulum_pend/ground_truth/imu", 1000, MsgCallback2);
  pos_quad = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback3);
  angular_velocity_quad = node.subscribe("/hummingbird_pendulum/ground_truth/imu", 1000, MsgCallback4);
  output_roll = node.advertise<std_msgs::Float32>("/swing_up/lqr_yaw_input", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/LQRRotational.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      {          
        time_stamp = ros::Time::now().toSec();                

        //calculate error
        error_pend_roll = des_pend_roll - pend_roll;
        error_pend_vel_roll = des_pend_vel_roll - pend_vel_roll;
        error_quad_yaw = des_quad_yaw - quad_yaw;
        error_quad_vel_yaw = des_quad_vel_yaw - quad_vel_yaw;

        //calculate control input
        u_yaw = K_pend_roll*error_pend_roll + K_pend_vel_roll*error_pend_vel_roll + K_quad_yaw*error_quad_yaw + K_quad_vel_yaw*error_quad_vel_yaw;

        //converting u_roll to rotational speed of motor
        if(u_yaw>0){
          u_yaw = sqrt(u_yaw / k);
        }
        else if(u_yaw<0){
          u_yaw = -sqrt(abs(u_yaw) / k);
        }

        //display information for debugging
        //ROS_INFO("pend_roll: %f",pend_roll);
        //ROS_INFO("pend_vel_roll: %f",pend_vel_roll);
        //ROS_INFO("quad_yaw: %f",quad_yaw);
        //ROS_INFO("quad_vel_yaw: %f",quad_vel_yaw);
        //ROS_INFO("u_yaw: %f",u_yaw);
        
        //saturate input to the system
        if(u_yaw > 200 || u_yaw != u_yaw){
        //  ROS_INFO("Thrust is: %f",u_yaw);
          u_yaw = 200;  
        }
        else if (u_yaw < -200){
          u_yaw = -200;
        }
        //Publish
        std_msgs::Float32 u;
        u.data = u_yaw;
        output_roll.publish(u);

        ros::spinOnce();
        rate.sleep();

        //write into file
        myfile << fixed << setprecision(4) << time_stamp << "\t" << pend_roll << "\t" << pend_vel_roll << "\t" << quad_yaw << "\t" << quad_vel_yaw << endl;
      }
  myfile.close();
  return 0;
};

// Callback Functions
//subscribing to pendulum states
void MsgCallback1(const geometry_msgs::PoseStamped& msg)
{  
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion pend;
    tf::quaternionMsgToTF(msg.pose.orientation, pend);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(pend).getRPY(pend_roll, pend_pitch, pend_yaw);
}

//subscribing to pendulum rotational velocity
void MsgCallback2(const sensor_msgs::Imu& msg)
{
  pend_vel_roll = msg.angular_velocity.x;
}

//subscribing to quadrotor states
void MsgCallback3(const geometry_msgs::PoseStamped& msg)
{
  //calculates quadrotor rotational pose
  tf::Quaternion quat;
  tf::quaternionMsgToTF(msg.pose.orientation, quat);
  tf::Matrix3x3(quat).getRPY(quad_roll, quad_pitch, quad_yaw);
}

//subscribing to quadrotor rotational velocity
void MsgCallback4(const sensor_msgs::Imu& msg)
{
  quad_vel_yaw = msg.angular_velocity.y;
}