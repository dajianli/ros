#pragma once

#include <ros/ros.h>
#include <control_toolbox/pid.h>
#include "wheelpwm.h"
#include "boost/shared_ptr.hpp"
#include "sp1s/runvelocity.h"

class RunVelocity
{
public:
	RunVelocity(ros::NodeHandle &n, int pinForward, int pinBackward)
	{
		target_speed_ = 0.0;
		pwm_command_ = 0;
  		boost::shared_ptr<CWheelpwm> wheel(new CWheelpwm(pinForward,pinBackward));
		m_wheel = wheel;

		// construct pid controller
		// load parameter from "p" "i" "d" , "i_clamp" "i_clamp_min" "i_clamp_max"
		if (!pid_controller_.init(n)){
			ROS_ERROR("MyController could not construct PID controller for RunVelocity");
		}
 		m_bPidEngaged = false;
	}

	/*
		http://blog.csdn.net/solstice/article/details/3066268
		boost::function<void(double)> VelocityReport;
		VelocityReport = boost::bind(&RunVelocity::OnVelocityReport, &object);
		VelocityReport(vel_detected);
	*/
	void OnVelocityReport(double vel_current)
	{
		if(!m_bPidEngaged)
		{
		//	ROS_INFO("!m_bPidEngaged  vel_current: %f",vel_current);
			return;
		}

		double error = fabs(target_speed_) - vel_current;
		if (fabs(error) < 0.005)
		{
			ROS_INFO("target: %f    current:%f    error:%f",
					target_speed_,vel_current,error);
			return;
		}
		//ROS_INFO("target: %f    current:%f     pmw:%ld",
		//				target_speed_,vel_current,pwm_command_);

		ros::Time _now = ros::Time::now();
		pwm_command_ = pid_controller_.computeCommand(
				error, _now - time_of_last_cycle_ );
		time_of_last_cycle_ = _now;

		//pid_controller_.printValues();


		// put new pwm on the wire
		RunByPWM();
	}

	// Run with the speed with direction
	// +ve. forward
	// -ve. backward
	void RunSpeedCommand(double speed)
		{
			pwm_command_ = 30;
			pid_controller_.reset();
			pid_controller_.setCurrentCmd((double)pwm_command_);
			target_speed_ = speed;
 			m_bPidEngaged = true;
 			time_of_last_cycle_ = ros::Time::now();
 			pid_controller_.printValues();
			RunByPWM();
		}

	bool IsTurningForward()
	{
		return target_speed_ > 0.0;
	}

private:
	void RunByPWM()
	{
		if(target_speed_ >= 0 )
			m_wheel->forward(pwm_command_);
		else
			m_wheel->backward(pwm_command_);
	}
	control_toolbox::Pid pid_controller_;
	ros::Time time_of_last_cycle_;
	ros::Subscriber runvel_sub;
	double target_speed_;
	long pwm_command_;
    bool m_bPidEngaged;
    boost::shared_ptr<CWheelpwm> m_wheel;

};
