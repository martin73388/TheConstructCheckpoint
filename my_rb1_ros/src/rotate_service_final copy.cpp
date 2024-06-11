//The C++ program will contain a ROS1 node that provides 
//a ROS Service named /rotate_robot. This service will make the robot rotate 
//for a specific number of degrees (defined by the user).

#include "ros/subscriber.h"
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <nav_msgs/Odometry.h>


class TurnRB1
{
  
    public:

        // ROS Objects
        ros::NodeHandle nh_;

        // ROS Services
        ros::ServiceServer my_service;

        // ROS Publishers
        ros::Publisher vel_pub;

         // ROS Subscribers
        ros::Subscriber odom_sub;
    
        // ROS Messages
        my_rb1_ros::Rotate rot_msg;
  
        TurnRB1()
        {
            my_service = nh_.advertiseService("/rotate_robot", &TurnRB1::my_callback, this);
            ROS_INFO("The Service /rotate_robot is READY");
            vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        }
        
        void turn()
        {
            vel_msg.angular.z = 0.2;
            vel_pub.publish(vel_msg);
        }
    
        void stop()
        {
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
        }
        
        bool my_callback(my_rb1_ros::Rotate::Request &req,
                         my_rb1_ros::Rotate::Response &res)
        {
            ROS_INFO("The Service /rotate_robot has been called");
            int i = 0;
            while (i < req.duration)
              {
                  move_in_circle();
                  usleep(1000000); // We set 1000000 because the time is set in microseconds
                  i++;
              }
            stop();
            res.result = "the robot have succesfull turn";
            ROS_INFO("Finished service /move_bb8_in_circle");
            return true;
        }
    
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_bb8_node");
  
  TurnRB1 turnrb1;

  ros::spin();
  
  return 0;
}