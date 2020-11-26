#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "eigen3/Eigen/Dense"
#include "tf/transform_datatypes.h"
#include <iostream>
#include "inverted_pendulum/ekf_msg.h"

using namespace Eigen;

MatrixXf x_hat(5,1), x_original(5,1), x_hat_prev(5,1), P(5,5), P_prev(5,5), Q(5,5), A(5,5), B(5,1), R(5,5), H(5,5), y(5,5), C(5,5), K(5,5), Z_hat(5,1), U(1,1);

/**
 * 
 *   -
 *  | x       |    
 *  | y       |
 *  | theta   | =    
 *  | omega   |
 *  | velocity|
 * 
*/


float sensor_x_pos, sensor_y_pos, sensor_omega, sensor_theta, sensor_speed;

void pose_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    sensor_x_pos = msg->pose.pose.position.x;
    sensor_y_pos = msg->pose.pose.position.y;
    sensor_theta = tf::getYaw(msg->pose.pose.orientation);
    printf("The y orietation of the bot is %f \n", sensor_y_pos);
    sensor_speed = msg->twist.twist.linear.x;
    sensor_omega = msg->twist.twist.angular.z;
    printf("%f\n", sensor_speed);
}

int main(int argc, char* argv[])
{
    MatrixXf Residual_covariance(5,5);
    MatrixXf identity(5,5);

    float assign;

    ros::init( argc, argv, "simple_publisher" );

    ros::NodeHandle n;

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000 );
    ros::Publisher ekf_pub = n.advertise<inverted_pendulum::ekf_msg>("/ekf_data", 1000);

    ros::Subscriber odometry_sub = n.subscribe( "/odom", 1000, pose_callback );
    ros::Rate loop_rate(10);

    geometry_msgs::Twist vel;
    inverted_pendulum::ekf_msg data;

    x_hat_prev(0,0) = x_hat_prev(1,0) = x_hat_prev(2,0) =  x_hat_prev(4,0) = 100;
    x_hat_prev(3,0) = 1.5;
    data.x_pose = x_hat_prev(0,0);
    data.velocity = x_hat_prev(1,0);

    // state covariance noise(Process noise)
    Q(0,0) = Q(1,1) = Q(2,2) = Q(3,3) = Q(4,4) = 0.01;

    //sensor conversion matrix
    H(0,0) = H(1,1) = H(2,2) = H(3,3) = H(4,4)= 1;


    // Estimate uncertainity(Co-variance) matrix
    P(0,0) = P(1,1) = P(2,2) = P(3,3) = P(4,4) =  10,000; // This is a best-guess estimate for the state covariance. This needs to be fixed.

    // input control matrix
    B(0,0) = 0.02;
    B(1,0) = 0.0001;

    // sensor uncertainity
    R(0,0) = R(1,1) = R(2,2) = R(3,3) = R(4,4) = 0.05;

    vel.linear.x = 1.25;
    vel.linear.y = 0;
    vel.angular.z = 0;

    while (ros::ok())
    {

        velocity_pub.publish(vel);
        ekf_pub.publish(data);

        // state transition matrix
        A(0,0) = A(1,1) = A(2,2) = A(3,3) = A(4,4) = 1;
        A(0,4) = 0.05 * cos(x_hat_prev(3,0)); // The publish rate is 20 hz that makes the delta t to be 0.05
        A(1,4) = 0.05 * sin(x_hat_prev(3,0));
        A(2,4) = 0.05;

        x_hat = A * x_hat_prev;

        std::cout<<""<<x_hat<<std::endl;


        P = A * P_prev * A.transpose() + Q;
        P_prev = P;

        Residual_covariance = H * P * H.transpose() + R;
        K = P * H.transpose() * Residual_covariance.inverse();


        // Ideally this should be done in a matrix representation.
        Z_hat(0, 0) = sensor_x_pos;
        Z_hat(1, 0) = sensor_y_pos;
        Z_hat(2, 0) = sensor_theta;
        Z_hat(3, 0) = sensor_omega;
        Z_hat(4, 0) = sensor_speed;

        x_original = x_hat + K * ( Z_hat - H * x_hat );

        data.x_pose = x_original(0,0);
        data.y_pose = x_original(1, 0);
        data.theta_pose = x_original(2,0);
        data.omega = x_original(3,0);
        data.velocity = x_original(4,0);

        x_hat_prev = x_original;

        std::cout<<"The updated matrix is:"<<x_original(0,0)<<std::endl;

        P  = ( identity - K*H ) * P;

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}