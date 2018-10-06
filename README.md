## pendulum

This ROS packages implements an inverted pendulum on a quadrotor. It was build and tested for Gazebo in ROS Indigo. The following ROS packages are needed:
1. joy
2. rotors_simulator
3. mav_comm
4. asctec_mav_framework

The urdf folder is adding the pendulum to the Ascending Technologies Hummingbird quadrotor in the `rotors_simulator` package. The files have to be copied into the `rotors_simulator/rotors_description/urdf` folder.

The quadrotor is able to swing up and balance the pendulum in a lateral and roational manner. A XBox controller with the ROS joy package is used to control the multirotor and switch between lateral and rotational mode. In addition the quadrotor can be brought back to its hovering position at the origin at 1m altitude.

To launch the code run
`roslaunch pendulum complete.launch`

The result of the package can be seen here
https://www.youtube.com/watch?v=4eOm25gVPY0