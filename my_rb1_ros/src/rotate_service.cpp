
#include <cstdio>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "ros/subscriber.h"
#include <geometry_msgs/Twist.h>
#include <my_rb1_ros/Rotate.h>
#include <angles/angles.h>
#include <tf/tf.h>



class TurnRB1
{
    public:

        // ROS Objects
        ros::NodeHandle nh;

        // ROS Services
        ros::ServiceServer my_service;

        // ROS Publishers
        ros::Publisher vel_pub;

         // ROS Subscribers
        ros::Subscriber odom_sub;
    
        // ROS Messages
        geometry_msgs::Twist vel_msg;
        my_rb1_ros::Rotate rot_msg;

        //Topic Variable
        float robotorientation;
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
        double target_angle = 9;
        double target_rad = 0.0;
        double kP = 0.5;  // Proportionnal to calculate the Angular Z velocity command
        


         TurnRB1()
        {
            
            my_service = nh.advertiseService("/rotate_robot", &TurnRB1::my_callback, this);
            ROS_INFO("The Service /rotate_robot is READY");
            vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
            odom_sub = nh.subscribe("odom", 1000, &TurnRB1::get_rotation, this);

   
        }


        void get_rotation(const nav_msgs::Odometry::ConstPtr& msg) {
            tf::Quaternion orientation_q(
                msg->pose.pose.orientation.x,
                msg->pose.pose.orientation.y,
                msg->pose.pose.orientation.z,
                msg->pose.pose.orientation.w
            );
            tf::Matrix3x3(orientation_q).getRPY(roll, pitch, yaw);
           // ROS_INFO("Yaw: %f", yaw);
        }

        bool my_callback(my_rb1_ros::Rotate::Request &req,
                         my_rb1_ros::Rotate::Response &res)
        {
            ROS_INFO("The Service /rotate_robot has been called");
    
            target_rad = angles::normalize_angle(req.degrees * M_PI / 180 + yaw); // Calculate relative target angle
            double diff = angles::shortest_angular_distance(yaw, target_rad);

        
            //target_rad = req.degrees * M_PI / 180;
            //double diff = target_rad - yaw;

            ros::Rate r(10); // 10 hz
            vel_msg.angular.z = 0.1;

            while(abs(vel_msg.angular.z) > 0.0008 )
            {
           

                //diff = target_rad - yaw;
                diff = angles::shortest_angular_distance(yaw, target_rad);
          
                ROS_INFO("the angle target in rad is %f", target_rad);
                ROS_INFO("the current angle in rad is %f", yaw);
                ROS_INFO("the angle difference in rad is %f", diff);
                ROS_INFO("the angle velocity is  %f", vel_msg.angular.z );
                ros::spinOnce(); // Refresh the robot orientation value
                turn();
                r.sleep();

            }
              
            stop();
            res.result = "Robot rotated succesfully";
            ROS_INFO("Finished service /rotate_robot");
            /* */
            return true;
            
        }


        void turn()
        {
            /*
            if (degrees >0)
            {
            vel_msg.angular.z = 0.2;
              //  ROS_INFO("le robot turn on the left");
                
            }
            else {
            vel_msg.angular.z = -0.2;
            ROS_INFO("le robot turn on the Right");
            } 
            */
            
            //vel_msg.angular.z = kP * (target_rad - yaw); 
            vel_msg.angular.z = kP * angles::shortest_angular_distance(yaw, target_rad);
         //   ROS_INFO("ang Z velocity :  %f" , vel_msg.angular.z);
            vel_pub.publish(vel_msg);
        }
    
        void stop()
        {
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
        }

    
}
;


int main(int argc, char** argv) {

    ros::init(argc, argv, "rotate_robot_node");
    
    TurnRB1 turnrb1;
    
    ros::spin();
    
    return 0;
}