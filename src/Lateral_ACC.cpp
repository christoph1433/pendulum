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

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Subscriber groundtruth_linear_acc_subscriber;
ros::Subscriber des_acc_y_subscriber;
ros::Publisher control_publisher;

// Global Variables
//PID gains
float KP_Acc_Y = 0.3; 
float KI_Acc_Y = 0.1; 
float KD_Acc_Y = 0.005;
//PID variables
float error_prior_acc_y = 0;
float error_acc_y = 0;
float integral_acc_y = 0;
float derivative_acc_y = 0;
//control input
float u_acc_y = 0;
//setup timestepc
float iteration_time = 0.001;
//desired lateral acceleration from energy lateral controller
float des_acc_y = 0;
//average filter
float acc_y[20] = {};
float sum = 0;
float avg_acc_y;
//Flag for initial loop
int flag = 0;
double begin_time = 0;
double duration = 0;
double time_stamp;

//callback functions
void MsgCallback(const sensor_msgs::Imu& msg);
void MsgCallback2(const std_msgs::Float32 msg);


int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;

  //defining arguments for subscribing and publishing to topics
  groundtruth_linear_acc_subscriber = node.subscribe("/hummingbird_pendulum/ground_truth/imu", 1000, MsgCallback);
  des_acc_y_subscriber = node.subscribe("/swing_up/des_y_acc", 1000, MsgCallback2);
  control_publisher = node.advertise<std_msgs::Float32>("/swing_up/y_input_acc", 1000);

  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Input_Acc_Y.txt", ios::trunc);

  ros::Rate rate(1000);
    while (node.ok())
      { 
        time_stamp = ros::Time::now().toSec();                

        //PID loop for acceleration y
        error_acc_y = des_acc_y - avg_acc_y;
        if(flag == 1){
          integral_acc_y = integral_acc_y + (error_acc_y*iteration_time);
          derivative_acc_y = (error_acc_y - error_prior_acc_y)/iteration_time;

          if(integral_acc_y!=integral_acc_y  || integral_acc_y-integral_acc_y != 0){integral_acc_y=0;}
        }
        u_acc_y = (KP_Acc_Y * error_acc_y + KI_Acc_Y * integral_acc_y + KD_Acc_Y * derivative_acc_y);
        error_prior_acc_y = error_acc_y;

        if(u_acc_y > 0.7 || u_acc_y != u_acc_y){
          u_acc_y = 0.7;  
        }
        else if (u_acc_y < -0.7){
          u_acc_y = -0.7;
        }

        //DEBUG
        // ROS_INFO("actual_acc_y: %f",avg_acc_y);
        // ROS_INFO("des_acc_y: %f",des_acc_y);
        // ROS_INFO("error_acc_y: %f",error_acc_y);
        // ROS_INFO("integral_acc_y: %f",integral_acc_y);
        // ROS_INFO("derivative_acc_y: %f",derivative_acc_y);
        // ROS_INFO("error_prior_acc_y: %f",error_prior_acc_y);
        // ROS_INFO("u_acc_y: %f",u_acc_y);

        //Publish
        std_msgs::Float32 u;
        u.data = u_acc_y;

        control_publisher.publish(u);
        ros::spinOnce();
        rate.sleep();

        flag += 1;
        if(flag>1){flag = 1;}

        //write into file
        myfile << fixed << setprecision(4) << time_stamp << "\t" << u_acc_y << "\t" << avg_acc_y << "\t" << des_acc_y <<endl;


      }
  myfile.close();
  return 0;
};

//subscribe and calculate y acceleration
void MsgCallback(const sensor_msgs::Imu& msg)
{    
    for(int i=20;i>0;i--)
    {
      acc_y[i] = acc_y[i-1];
    }
    acc_y[0] = msg.linear_acceleration.y;
    sum = 0;
    for(int i=0;i<20;i++)
    {
      sum=sum+acc_y[i];
    }
    avg_acc_y = sum/20;
}

//subscibe to desied acc from lateral energy control
void MsgCallback2(const std_msgs::Float32 msg)
{
  des_acc_y = msg.data;
}
