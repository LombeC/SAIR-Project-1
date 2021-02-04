//
// Created by lombe
//

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "turtlesim/Pose.h"


ros::Publisher pub; // publishes the velocity
ros::Subscriber sub; //

turtlesim::Pose pose; //current pose of the turtle

void poseCallback(const turtlesim::Pose::ConstPtr& msg)
{
    //update the pose in each iteration
    pose.x = msg->x;
    pose.y = msg->y;
    pose.theta = msg->theta;
    pose.linear_velocity = msg->linear_velocity;
    pose.angular_velocity = msg->angular_velocity;
    ROS_INFO("x: %.2f, y: %.2f, theta: %.2f, lin_vel: %.2f, ang_vel: %.2f", pose.x, pose.y, pose.theta, pose.linear_velocity, pose.angular_velocity);
}

//helper to move and rotate the turtle
//inspiration take from tutorials http://wiki.ros.org/turtlesim/Tutorials
void move_turtle(double speed, double distance);
void rotate_turtle(double speed, double angle_in_radians, bool clockwise);
void draw_d();

int main(int argc, char **argv)
{

    //initialize the node
    ros::init(argc, argv, "du_chileshe_lombe");
    ros::NodeHandle node;

    pub = node.advertise <geometry_msgs::Twist>("turtle1/cmd_vel", 10);
    sub = node.subscribe("turtle1/pose", 1000, poseCallback); //subscriber receives turtle's pose data


    ROS_INFO("Starting to draw D");
    draw_d();
    return 0;
}


void draw_d()
{
    double angle_speed = 1;
    double movement_speed = 2;

    move_turtle(movement_speed, 2); // go forward
    rotate_turtle(angle_speed, 0.785, false); //45 degree turn
    move_turtle(movement_speed, 1);
    rotate_turtle(angle_speed, 0.523599, false);
    move_turtle(movement_speed, 2);
    rotate_turtle(angle_speed, 0.785, false);
    move_turtle(movement_speed, 0.76);
    rotate_turtle(angle_speed, 0.523599, false);
    move_turtle(movement_speed, 1);
    rotate_turtle(angle_speed, 1.3708, false);
    move_turtle(movement_speed, 0.5);
    rotate_turtle(angle_speed, 1.3708, false);
    move_turtle(movement_speed, 0.2);
    rotate_turtle(angle_speed, 1.3708, true);
    move_turtle(movement_speed, 2);
    rotate_turtle(angle_speed, 1.3708, true);
    move_turtle(movement_speed, 0.2);
    rotate_turtle(angle_speed, 1.3708, false);
    move_turtle(movement_speed, 0.5);
}

//move the turtle forward at a certain speed for a certain distance
void move_turtle(double speed, double distance){
    geometry_msgs::Twist msg; // publish the speed with this message

    msg.linear.x = abs(speed); // move forward
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    double start = ros::Time::now().toSec(); // use a timer to determine when distance has been covered
    double distance_moved = 0.0;
    ros::Rate rate(10);
    while(distance_moved < distance) //keep moving the turtle forward until we hit out target distance
    {
        pub.publish(msg);
        double finish = ros::Time::now().toSec();
        distance_moved = speed * (finish - start); // calculate the distance moved in each iteration
        //minimal close loop control to improve accuracy
        if (pose.x <= 0 || pose.x >= 11)
        {
            ROS_WARN("Hitting the wall");
            speed -= 0.01; // reduce the speed of turtle if you hit the wall
        }

        ros::spinOnce();
        rate.sleep();
    }
    msg.linear.x = 0; //bring turtle to stop after moving target distance
    pub.publish(msg);
}

void rotate_turtle (double speed, double angle_in_radians, bool clockwise)
{
    geometry_msgs::Twist msg;

    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;

    msg.angular.x = 0;
    msg.angular.y = 0;

    if (clockwise) {
        msg.angular.z = -abs(speed); // turn the turtle at a certain pace
    } else {
        msg.angular.z = abs(speed);
    }

    double start = ros::Time::now().toSec();
    double angle_rotated = 0.0;
    ros::Rate rate(10);
    while(angle_rotated < angle_in_radians) //keep rotating the turtle until we hit our target angle
    {
        pub.publish(msg);
        double finish = ros::Time::now().toSec();
        angle_rotated = speed * (finish - start);
        ros::spinOnce();
        rate.sleep();
    }

    msg.angular.z = 0; //turtle stops after reaching angle
    pub.publish(msg);
}

