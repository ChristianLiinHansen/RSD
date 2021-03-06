#include "ros/ros.h"
#include <rsd_project/bricks_to_robot.h>   	// Added to include my custum msg file,bricks_to_robo.msg
#include <geometry_msgs/Pose.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using namespace std;
using namespace cv;

// The r_and_theta callback function
void brickPoseCallBack(const rsd_project::bricks_to_robot::ConstPtr& msg)
//void brickPoseCallBack(const rsd_project::bricks_to_robot& header)
{
    // Receive the pose
    geometry_msgs::Pose pose;
    pose = msg-> pose;
    cout << "\n------------------ Brick received ------------------\n" << pose << "\n";

    // Receive the timestamp
    double receiveTime;
    receiveTime = msg->header.stamp.toSec();

    // Receive the ID
    string color;
    color = msg->header.frame_id;
    cout << "Color: " << color << endl;

    // Calculate the differnce in time from send to received
    double currentTime = ros::Time::now().toSec();
    double offset;
    offset = currentTime-receiveTime;
    cout << setprecision(15);
    cout <<"offset from stamped in legoDetection to be accessable in listener_nodeRSD: " << receiveTime << endl;

	
    //ros::NodeHandle n;
	// Publisher cmd_velo
    //ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener_nodeRSD");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
	
    //rsd_project::bricks_to_robot:: num msg;

    //Subscriber lego_pose
    ros::Subscriber lego_pose_sub = n.subscribe("/lego_pose", 1000, brickPoseCallBack);
	
    // Publisher cmd_vel
    //ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

	while (ros::ok())
	{
		// Publish the r and theta trough ROS
//		twist.linear.x = 0.2;
//		twist.linear.y = 0.0;
//		twist.linear.z = 0.0;
//		
//		twist.angular.x = 0.0;
//		twist.angular.y = 0.0;
//		twist.angular.z = 0.0;
		
		//cout << "Do we come to this point?" << endl;
		//msg.number = 23;
		//test_pub.publish(msg);
		
		// And then we send it on the test_pub topic
		//cmd_vel_pub.publish(twist);
		
        //loop_rate.sleep();
		ros::spinOnce();
	}	
	return 0;
}
