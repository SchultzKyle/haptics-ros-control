#include <stdio.h>
#include <math.h>
#include "dhdc.h"
#include <chrono>
#include <thread>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/thread/thread.hpp>

int main (int argc,char **argv) {

  // parameters:
  double origin[3] = {0.0, 0.0, 0.0}; // origin
  double K[3] = {100.0, 100.0, 100.0}; // haptic device spring constants to origin
  double B[3] = {12.5, 12.5, 12.5}; // haptic device damping constants to origin
  double sat_force = 18.0; // saturation force
  double Kv = 0.01; // haptic position to end effector velocity gain
  double gripper_force = 1.0; // gripper force (open stops motion, force to keep open)
  double gripper_closed = 0.001; // defines gripper reading corresponding to start of motion
  double camera_pos_lim = 0.39;

  double last_camera_pos = 0.0;
  double p[3]; // position
  double o[3]; // orientation
  double v[3]; // velocity
  double f[3]={0.0, 0.0, 0.0}; // force
  double gripper;

  //ROS node/sub/pub
  ros::init(argc,argv,"sigma_vel_cmd");
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Publisher vel_pub = n->advertise<geometry_msgs::Twist>("cmd_vel_linear",10); // linear velocities
  //ros::Publisher camera_pub = n->advertise<std_msgs::Float64>("/barc_arm/joint6_position_controller/command",1000); // for gazebo simulation camera

  ros::Rate loop_rate(1000);
  
  //connect to device
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }
  printf ("%s device detected\n\n", dhdGetSystemName());
  dhdEnableExpertMode();  
  dhdEnableForce (DHD_ON);

  //variables
  dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
  dhdGetOrientationRad(&(o[0]), &(o[1]), &(o[2]));
  dhdSetForceAndTorqueAndGripperForce(0.0,0.0,0.0,0.0,0.0,0.0,0.0); // initialize to zero


  while(ros::ok()) {
    // printf("x: %lf, y: %lf, z: %lf\n",dx[0], dx[1], dx[2]); // print variables example
    
    //read haptic device states
    dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
    dhdGetOrientationRad(&(o[0]), &(o[1]), &(o[2]));
    dhdGetLinearVelocity (&(v[0]), &(v[1]), &(v[2]));
    dhdGetGripperGap(&gripper);

    // set haptic device force:
    for(int i=0; i<3; i++) {
      f[i] = K[i]*(origin[i]-p[i]) - B[i]*v[i]; // spring-damper to origin
      f[i] = std::min(std::max(f[i], -sat_force), sat_force); // force saturation
    }
    dhdSetForceAndTorqueAndGripperForce(f[0],f[1],f[2],0.0,0.0,0.0,gripper_force);
    
    //create ros message 
    geometry_msgs::Twist msg;
    geometry_msgs::Vector3 vel;
    if(gripper < gripper_closed) {
      // set end effector xyz velocity
      vel.x = -Kv*p[0];
      vel.y = -Kv*p[1];
      vel.z = Kv*p[2];
    } else {
      vel.x = 0;
      vel.y = 0;
      vel.z = 0;
    }    
  
    // publish ros message
    msg.linear = vel;
    vel_pub.publish(msg);
    loop_rate.sleep();
    ros::spinOnce();
  }

  dhdSetForceAndTorqueAndGripperForce(0.0,0.0,0.0,0.0,0.0,0.0,0.0); // turn off force
  return 0;
  
}
