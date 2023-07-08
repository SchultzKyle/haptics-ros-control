#include <stdio.h>
#include <math.h>
#include "dhdc.h"
#include <chrono>
#include<thread>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <boost/thread/thread.hpp>

int main (int argc,char **argv)
{

  // parameters:
  double origin[3] = {0.0, 0.0, 0.0}; // origin
  double K[3] = {100.0, 100.0, 100.0}; // haptic device spring constants to origin
  double B[3] = {10.0, 10.0, 10.0}; // haptic device damping constants to origin
  double sat_force = 12.0; // saturation force
  double Kv = 1.0; // haptic position to end effector velocity gain
  double camera_pos_lim = 0.39;

  double last_camera_pos = 0.0;
  double p[3]; // position
  double o[3]; // orientation
  double v[3]; // velocity
  double force[3]={0.0, 0.0, 0.0};
  double gripper;

   //ROS STUFF
  ros::init(argc,argv,"omega_cmd_vel");
  ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
  ros::Publisher vel_pub = n->advertise<geometry_msgs::Twist>("/barc_arm/xyz_cmd_vel",1000);
  ros::Publisher camera_pub = n->advertise<std_msgs::Float64>("/barc_arm/joint6_position_controller/command",1000);

  ros::Rate loop_rate(1000);
  
  //haptic stuff
  
  if (dhdOpen () < 0) {
    printf ("error: cannot open device (%s)\n", dhdErrorGetLastStr());
    dhdSleep (2.0);
    return -1;
  }

  // identify device
  printf ("%s device detected\n\n", dhdGetSystemName());
  //dhdEnableExpertMode();  
  dhdEnableForce (DHD_ON);
  dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
  dhdGetOrientationRad(&(o[0]), &(o[1]), &(o[2]));
  dhdSetForceAndTorqueAndGripperForce(0.0,0.0,0.0,0.0,0.0,0.0,0.0);
  while(ros::ok())
  {
    //printf("x: %lf, y: %lf, z: %lf\n",dx[0], dx[1], dx[2]);
    
    //read haptic device states
    dhdGetPosition(&(p[0]), &(p[1]), &(p[2]));
    dhdGetOrientationRad(&(o[0]), &(o[1]), &(o[2]));
    dhdGetLinearVelocity (&(v[0]), &(v[1]), &(v[2]));
    dhdGetGripperGap(&gripper);

    // printf("gripper count: %lf \n", gripper);
    // printf("x: %lf, y: %lf, z: %lf\n",p[0], p[1], p[2]);

    // set haptic device force:
    for(int i=0; i<3; i++){
      force[i] = K[i]*(origin[i]-p[i]) - B[i]*v[i];
      force[i] = std::min(std::max(force[i], -sat_force), sat_force); // saturation
    }
    dhdSetForceAndTorqueAndGripperForce(force[0],force[1],force[2],0.0,0.0,0.0,1);
    // printf("fx: %lf, fy: %lf, fz: %lf\n",force[0], force[1], force[2]);
    
    // to turn off gripper force
    // dhdSetForceAndTorqueAndGripperForce(force[0],force[1],force[2],0.0,0.0,0.0,0.0);
    
    //create ros message 
    geometry_msgs::Twist msg;
    geometry_msgs::Vector3 vel;
    std_msgs::Float64 camera_msg;
    if(gripper < 0.001) {
      // set end effector xyz velocity
      vel.x = Kv*p[0];
      vel.y = -Kv*p[1];
      vel.z = Kv*p[2];
      // set camera position to wrist tilt
      camera_msg.data = -o[1];

    } else {
      vel.x = 0;
      vel.y = 0;
      vel.z = 0;
      camera_msg.data = last_camera_pos;
    }    

    // to turn off gripper command
    // vel.x = Kv*p[0];
    // vel.y = -Kv*p[1];
    // vel.z = Kv*p[2];

    // set camera position to wrist tilt
    camera_msg.data = -o[1];

    //limit joint to avoid jitter in gazebo
    if (camera_msg.data > camera_pos_lim) {
      camera_msg.data = camera_pos_lim;
    }
    else if (camera_msg.data < -camera_pos_lim) {
      camera_msg.data = -camera_pos_lim;
    }
  
    // publish ros message
    last_camera_pos = camera_msg.data;
    msg.linear = vel;
    vel_pub.publish(msg);
    camera_pub.publish(camera_msg);
    loop_rate.sleep();
    ros::spinOnce();
  }

  dhdSetForceAndTorqueAndGripperForce(0.0,0.0,0.0,0.0,0.0,0.0,0.0);
  return 0;
  
}
