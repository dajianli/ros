// follow tutoral http://wiki.ros.org/navigation/Tutorials/RobotSetup/Odom

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <wiringPi.h>
#include <sstream>
#include "boost/shared_ptr.hpp"
#include <geometry_msgs/Twist.h>
#include "sp1s/velocity.h"
#include "sp1s/wheels_dir.h"
#include "RunVelocity.h"
#include "veldetect.h"
#include "odom_publish.h"

boost::shared_ptr<RunVelocity> m_leftwheel;
boost::shared_ptr<RunVelocity> m_rightwheel;
CVeldetect* m_pVeldetect;
COdomPublish* m_odomPub;
bool m_bleft_forward;
bool m_bright_forward;

void cmd_vel_callback(const geometry_msgs::Twist& vel_cmd)
{
	ROS_INFO("I heard Twist: [%f]", vel_cmd.linear.x);
	double  Lwheelspeed, Rwheelspeed;
	m_odomPub->odom_to_speed(vel_cmd.linear.x, vel_cmd.linear.y,
			vel_cmd.angular.z,
			Lwheelspeed, Rwheelspeed);
	m_leftwheel->RunSpeedCommand(Lwheelspeed);
	m_rightwheel->RunSpeedCommand(Rwheelspeed);
	m_pVeldetect->Reset();
	m_bleft_forward = Lwheelspeed > 0 ? true : false;
	m_bright_forward = Rwheelspeed > 0 ? true : false;
}

void wheels_dir_callback(const sp1s::wheels_dir& dir)
{
	m_bleft_forward = dir.left_forward;
	m_bright_forward = dir.right_forward;
}
int main(int argc, char **argv)
{
	ros::init(argc, argv, "base_control");
	ros::NodeHandle n;

	ros::Subscriber cmd_vel_sub = n.subscribe("/cmd_vel", 10, cmd_vel_callback);
	ros::Subscriber wheels_dir_sub = n.subscribe("/wheels_dir", 10, wheels_dir_callback);
	wiringPiSetup();

	int left_forward, left_backward, right_forward, right_backward;
	n.param("left_forward", left_forward, 1);
	n.param("left_backward", left_backward, 4);
	n.param("right_forward", right_forward, 6);
	n.param("right_backward", right_backward, 5);
/*
	ROS_INFO("right_forward %d", left_forward);
	ROS_INFO("left_backward %d", left_backward);
	ROS_INFO("right_forward %d", right_forward);
	ROS_INFO("right_backward %d", right_backward);

	double p,i,d;
	n.getParam("p", p);
	n.getParam("i", i);
	n.getParam("d", d);
	ROS_INFO("p: %f    i:%f     d:%f", p,i,d);
*/
	bool bRemoteControl;
	n.param("remote", bRemoteControl, true);
	if(!bRemoteControl)
	{
	//	RunVelocity leftwheel(n, left_forward, left_backward);
	//	RunVelocity rightwheel(n, right_forward, right_backward);
		m_leftwheel = boost::shared_ptr<RunVelocity>(new RunVelocity(n, left_forward, left_backward));
		m_rightwheel = boost::shared_ptr<RunVelocity>(new RunVelocity(n, right_forward, right_backward));
		ROS_INFO("remote disabled, ready to run on /cmd_vel");
	}

	int left_vel_pin, right_vel_pin;
	n.param("left_vel_pin", left_vel_pin, 28);
	n.param("right_vel_pin", right_vel_pin, 29);
	CVeldetect veldetect;
	veldetect.init(left_vel_pin, right_vel_pin);
	m_pVeldetect = &veldetect;

	COdomPublish odomPub;
	odomPub.Init(n);
	m_odomPub = &odomPub;

	ros::Rate loop_rate(10);
	double vel_left = 0.0;
	double vel_right = 0.0;
	double speed_left = 0.0;
	double speed_right = 0.0;
	m_bleft_forward = true;
	m_bright_forward = true;

	while (ros::ok())
	{
		//set the velocity
		vel_left = veldetect.computeLeftVel();
		vel_right = veldetect.computeRightVel();
		
		if(!bRemoteControl)
		{
			m_leftwheel->OnVelocityReport(vel_left);
			m_rightwheel->OnVelocityReport(vel_right);
		}
		veldetect.Reset();
		/*
		ROS_INFO("vel_left %f", vel_left);
		ROS_INFO("vel_right %f", vel_right);
		*/
		//publish the odom message
		speed_left = m_bleft_forward ? vel_left : -vel_left;
		speed_right = m_bright_forward ? vel_right : -vel_right;
		odomPub.SpeedCallback(speed_left, speed_right);
		ros::spinOnce();
		loop_rate.sleep();
	}
}
