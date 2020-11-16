#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "eigen3/Eigen/Dense"
#include <iostream>

using namespace Eigen;

MatrixXf x_hat(2,1), x_orginal(2,1), x_hat_prev(2,1), P(2,2), P_prev(2,2), Q(2,2), A(2,2), B(2,1), R(2,2), H(2,2), y(2,2), C(2,2), K(2,2), Z_hat(2,1), U(1,1);

double x_pos, velocity;
void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    x_pos = msg->pose.pose.position.x;
    velocity = msg->twist.twist.linear.x;
    printf("%f\n", velocity);
}

int main(int argc, char* argv[])
{
    MatrixXf inverse(2,2);
    MatrixXf identity(2,2);

    ros::init( argc, argv, "simple_publisher" );

    ros::NodeHandle n;

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100 );

    ros::Subscriber odometry_sub = n.subscribe( "/odom", 1000, pose_callback );
    ros::Rate loop_rate(50);

    geometry_msgs::Twist velocity;

    x_hat_prev(0,0) = x_hat_prev(1,0) = 5;
    
    // state transition matrix
    A(0,0) = A(1,1) = 1;
    A(0,1) = 0.02;
    A(1,0) = 0;

    // state covariance noise
    Q(0,0) = Q(1,1) = 0.01;

    //sensor conversion matrix
    H(0,0) = H(1,1) = 1;
    H(1,0) = H(0,1) = 0;

    // state covariance
    P(0,0) = P(1,1) = 10,000;
    P(0,1) = P(1,0) = 0;

    // control matrix
    B(0,0) = 0.02;
    B(1,0) = 0.0001;

     // sensor uncertainity    
    R(0,0) = R(1,1) = 0.05;
    R(0,1) = R(1,0) = 0;

    velocity.linear.x = 3;
    velocity.linear.y = 0;
    velocity.angular.z = 0;

    while (ros::ok())
    {
        velocity_pub.publish(velocity);

        x_hat = A * x_hat_prev + B * U;
        P = A * P_prev * A.transpose()+ Q;
        K = H * P * H.transpose()*inverse.inverse();

        Z_hat(0, 0) = x_pos;
        Z_hat(1, 0) = velocity;
        x_original = x_hat + K * ( Z_hat - H * x_hat );

        P  = ( identity - K*H ) * P; 
        


        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}