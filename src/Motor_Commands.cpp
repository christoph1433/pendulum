#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Accel.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/Joy.h"
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


// global publisher and subscriber 
ros::Subscriber thrust_subscriber;
ros::Subscriber roll_subscriber;
ros::Subscriber pitch_subscriber;
ros::Subscriber yaw_subscriber;
ros::Subscriber roll_acc_subscriber;
ros::Subscriber lqr_roll_subscriber;
ros::Subscriber yaw_acc_subscriber;
ros::Subscriber lqr_yaw_subscriber;
ros::Subscriber angle_pendulum;
ros::Subscriber reduce_swing;
ros::Subscriber joystick;
ros::Subscriber pos_quad;
ros::Publisher control_publisher;

// setup Global Variables
//set controls inputs
float u_thrust = 0;
float u_roll = 0;
float u_pitch = 0;
float u_yaw = 0;
float u_roll_lqr_linear, u_roll_acc;
float u_thrust_position, u_roll_position, u_pitch_position, u_yaw_position, u_yaw_acc, u_yaw_lqr, u_roll_reduce_swing;
//set motors rotational speeds
float motor_1, motor_2, motor_3, motor_4;
//setup timestep
float begin_time = 0, end_time = 0, iteration_time = 0.001;
//pendulum parameters
float pend_roll, pend_pitch, pend_yaw;
//quadrotor parameters
float quad_x, quad_y, quad_z, quad_roll, quad_pitch, quad_yaw;
//introduce Flightmode
int FlightMode;
  
//callback functions
void MsgCallback1(const std_msgs::Float32);
void MsgCallback2(const std_msgs::Float32);
void MsgCallback3(const std_msgs::Float32);
void MsgCallback4(const std_msgs::Float32);
void MsgCallback5(const std_msgs::Float32);
void MsgCallback6(const std_msgs::Float32);
void MsgCallback7(const std_msgs::Float32);
void MsgCallback8(const std_msgs::Float32);
void MsgCallback9(const geometry_msgs::PoseStamped& msg);
void MsgCallback10(const std_msgs::Float32);
void MsgCallback11(const sensor_msgs::Joy& msg);
void MsgCallback12(const geometry_msgs::PoseStamped& msg);

int main(int argc, char** argv){
  ros::init(argc, argv, "my_listener");

  ros::NodeHandle node;
  
  //defining arguments for subscribing and publishing to topics
  thrust_subscriber = node.subscribe("/swing_up/thrust_input", 1000, MsgCallback1);
  roll_subscriber = node.subscribe("/swing_up/roll_input", 1000, MsgCallback2);
  pitch_subscriber = node.subscribe("/swing_up/pitch_input", 1000, MsgCallback3);
  yaw_subscriber = node.subscribe("/swing_up/yaw_input", 1000, MsgCallback4);
  roll_acc_subscriber = node.subscribe("/swing_up/roll_input_acc", 1000, MsgCallback5);
  lqr_roll_subscriber = node.subscribe("/swing_up/lqr_roll_input", 1000, MsgCallback6);
  yaw_acc_subscriber = node.subscribe("/swing_up/yaw_input_acc", 1000, MsgCallback7);
  lqr_yaw_subscriber = node.subscribe("/swing_up/lqr_yaw_input", 1000, MsgCallback8);
  angle_pendulum = node.subscribe("/hummingbird_pendulum_pend/ground_truth/pose", 1000, MsgCallback9);
  reduce_swing = node.subscribe("/swing_up/lqr_reduce_swing_roll_input", 1000, MsgCallback10);
  joystick = node.subscribe("/joy", 1000, MsgCallback11);
  pos_quad = node.subscribe("/hummingbird_pendulum/ground_truth/pose", 1000, MsgCallback12);
  control_publisher = node.advertise<mav_msgs::Actuators>("/hummingbird_pendulum/command/motor_speed", 1000);


  //open file
  ofstream myfile;
  myfile.open ("/home/marhes-lab/Desktop/Record File/Motor_Commands.txt", ios::trunc);

  ros::Rate rate(1000);
  	while (node.ok())
  		{
        time_stamp = ros::Time::now().toSec();           			
        
        if (FlightMode == 1){                         //Hovering Mode
          u_thrust = u_thrust_position;
          u_roll = u_roll_reduce_swing;
          u_pitch = u_pitch_position;
          u_yaw = u_yaw_position;
        }

        if (FlightMode == 2 || FlightMode == 3){      //Linear Mode
            if(pend_roll < -0.7 || pend_roll > 0.7){  //Balancing Linear
            u_roll = u_roll_acc;
            u_yaw = u_yaw_position;
            u_thrust = u_thrust_position;
            u_pitch = u_pitch_position;
            //ROS_INFO("swing_up");
            //ROS_INFO("pend_roll: %f",pend_roll);
            FlightMode == 2;
          }
          else{                                        //Swing-up Linear
            u_roll = u_roll_lqr_linear;
            u_yaw = u_yaw_position;
            u_thrust = u_thrust_position;
            u_pitch = u_pitch_position;
            //ROS_INFO("balance");
            //ROS_INFO("pend_roll: %f",pend_roll);
            FlightMode = 3;
          }
        }

        if (FlightMode == 4 || FlightMode == 5){        //Rotational Mode
            if(pend_roll < -0.2 || pend_roll > 0.2){    //Balancing Rotational
            u_thrust = u_thrust_position;
            u_roll = u_roll_position;
            u_pitch = u_pitch_position;
            u_yaw = u_yaw_acc;
            //ROS_INFO("swing_up");
            //ROS_INFO("pend_roll: %f",pend_roll);
            FlightMode = 4;
          }
          else{                                         //Swing-up Rotational
            u_thrust = u_thrust_position;
            u_roll = u_roll_position;
            u_pitch = u_pitch_position;
            u_yaw = u_yaw_lqr;
            //ROS_INFO("balance");
            //ROS_INFO("pend_roll: %f",pend_roll);
            FlightMode = 5;
          }
        }
        
  			//calculate motor angular velocities
        if(flag == 3){
          motor_1 = 456+u_thrust-u_pitch+u_yaw;
          motor_2 = 456+u_thrust+u_roll-u_yaw;
          motor_3 = 456+u_thrust+u_pitch+u_yaw;
          motor_4 = 456+u_thrust-u_roll-u_yaw;
        }
        // display motor commands to terminal
        // ROS_INFO("MOTOR 1: %f",motor_1);
        // ROS_INFO("MOTOR 2: %f",motor_2);
        // ROS_INFO("MOTOR 3: %f",motor_3);
        // ROS_INFO("MOTOR 4: %f",motor_4);
        
        //saturate motor commands
        if (motor_1 > 1250 ){
          motor_1 = 1250;
        }
        if (motor_1 < 200 || motor_1 != motor_1){
          motor_1 = 200;
        }

        if (motor_2 > 1250){
          motor_2 = 1250;
        }
        if (motor_2 < 200 || motor_2 != motor_2){
          motor_2 = 200;
        }
        if (motor_3 > 1250){
          motor_3 = 1250;
        }
        if (motor_3 < 200 || motor_3 != motor_3){
          motor_3 = 200;
        }
        if (motor_4 > 1250){
          motor_4 = 1250;
        }
        if (motor_4 < 200 || motor_4 != motor_4){
          motor_4 = 200;
        }


  			//Publish
  			mav_msgs::Actuators u;
  			u.angular_velocities = {motor_1, motor_2, motor_3, motor_4};
  			control_publisher.publish(u);

        ros::spinOnce();
    		rate.sleep();

        //write into file
        myfile << fixed << setprecision(4) << time_stamp << "\t" << FlightMode << "\t"  << quad_x << "\t"  << quad_y << "\t"  << quad_z <<  "\t" << quad_roll << "\t"  << quad_pitch << "\t"  << quad_yaw << "\t"  << pend_roll <<  "\t" << motor_1 << "\t"  << motor_2 << "\t"  << motor_3 << "\t"  << motor_4 << endl;
  		}
  //close file
  myfile.close();
  return 0;
};

// Callback Functions
// thrust input for altitude
void MsgCallback1(const std_msgs::Float32 msg)
{
  u_thrust_position = msg.data;             
}

//roll input for position control
void MsgCallback2(const std_msgs::Float32 msg)
{
  u_roll_position = msg.data;
}

//pitch input for position control
void MsgCallback3(const std_msgs::Float32 msg)
{
  u_pitch_position = msg.data;
}

//heading input for position control
void MsgCallback4(const std_msgs::Float32 msg)
{
  u_yaw_position = msg.data;
}

//roll input for swing-up
void MsgCallback5(const std_msgs::Float32 msg)
{
  u_roll_acc = msg.data;
}

//roll input for balancing
void MsgCallback6(const std_msgs::Float32 msg)
{
  u_roll_lqr_linear = msg.data;
}

//yaw input for swing-up
void MsgCallback7(const std_msgs::Float32 msg)
{
  u_yaw_acc = msg.data;
}

//yaw input for balancing
void MsgCallback8(const std_msgs::Float32 msg)
{
  u_yaw_lqr = msg.data;
}

//subscribing to pendulum states
void MsgCallback9(const geometry_msgs::PoseStamped& msg)
{  
    // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion pend;
    tf::quaternionMsgToTF(msg.pose.orientation, pend);

    // the tf::Quaternion has a method to access roll pitch and yaw
    tf::Matrix3x3(pend).getRPY(pend_roll, pend_pitch, pend_yaw);
}

//roll input for reducing swing of pendulum in downwards position
void MsgCallback10(const std_msgs::Float32 msg)
{
  u_roll_reduce_swing = msg.data;
}

//subsribing to XBox controller to switch flight modes
void MsgCallback11(const sensor_msgs::Joy& msg)
{
  if(msg.buttons[0] == 1){
    FlightMode = 1;
    ROS_INFO("Takeoff");
  }
  if(msg.buttons[1] == 1){
    FlightMode = 2;
    ROS_INFO("Rotational");
  }
  if(msg.buttons[2] == 1){
    FlightMode = 4;
    ROS_INFO("Linear");
  }
  if(msg.buttons[3] == 1){
    FlightMode = 6;
    ROS_INFO("Landing");
  }
}

//subscribing to quadrotor states
void MsgCallback12(const geometry_msgs::PoseStamped& msg)
{
  // the incoming geometry_msgs::Quaternion is transformed to a tf::Quaterion
    tf::Quaternion quat_quad;
    tf::quaternionMsgToTF(msg.pose.orientation, quat_quad);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    tf::Matrix3x3(quat_quad).getRPY(quad_roll, quad_pitch, quad_yaw);

    //position in world frame
    quad_x = msg.pose.position.x;
    quad_y = msg.pose.position.y;
    quad_z = msg.pose.position.z;
}