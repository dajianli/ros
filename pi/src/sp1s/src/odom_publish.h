#pragma once
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

class COdomPublish  
{  
public:  
  void Init(ros::NodeHandle& n)
  {
	  x_ = 0.0;  
	  y_ = 0.0;  
	  th_ = 0.0;  
	         
	  n.param("left_vel2ang_ratio",left_theta_to_speed,0.76);
	  n.param("right_vel2ang_ratio",right_theta_to_speed,0.77);
	  //freq_slave = 10;
	  last_time_ = ros::Time::now();
	  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
  }
  
  double yawrate_to_speed(double yawrate)
    {
  	  double theta_to_speed = 1;
         if (yawrate > 0)  
            theta_to_speed = right_theta_to_speed;//右转系数  
         else
            theta_to_speed = left_theta_to_speed; //左转系数  
                        
  	   //yawrate ：rad/s 
         double x = yawrate  / theta_to_speed ;
         return x;
    }
  
  void odom_to_speed(double cmd_twist_x, double cmd_twist_y, double cmd_twist_rotation,
		  double& Lwheelspeed, double& Rwheelspeed)  
  {
       double  cent_speed = sqrt(pow(cmd_twist_x, 2) + pow(cmd_twist_y, 2));
	   double yawrate2 = yawrate_to_speed(cmd_twist_rotation);
            
       Lwheelspeed = cent_speed - yawrate2/2;
          
       Rwheelspeed = cent_speed + yawrate2/2;
  }
  
	void speed_to_odom(float vel_left, float vel_right,
		           float& vx, float& vy, float& rad_z)
	{  
	    float delta_speed = vel_right - vel_left;
	    float theta_to_speed = 0;
	    if (delta_speed < 0 ) 
		theta_to_speed = right_theta_to_speed; //右转系数  
	    	else
		theta_to_speed = left_theta_to_speed;//左转系数  
		  
	    //*比例系数是将单位时间内的左右轮位移差（速度差）转化旋转的角度增量，再 * freqency，得到旋转角速度  
	    rad_z = delta_speed  * theta_to_speed;// * freq_slave;
	    vx = (vel_right + vel_left)/2.0;
	    vy = 0.0;
	}

	void SpeedCallback(float speed_left, float speed_right)
	{
	  ros::Time current_time_ = ros::Time::now();  
	  float vx=0.0, vy=0.0, v_rad=0.0;
	  speed_to_odom(speed_left,speed_right, vx, vy, v_rad);

	  //compute odometry in a typical way given the velocities of the robot
	  //double dt = 1 / freq_slave;
	  double dt =  (current_time_ - last_time_).toSec();
	  last_time_ = current_time_;
	  double delta_th = v_rad * dt; 
	  double delta_x = (vx * cos(th_) - vy * sin(th_)) * dt;  
	  double delta_y = (vx * sin(th_) + vy * cos(th_)) * dt;
	  
	  x_ += delta_x;  
	  y_ += delta_y;  
	  th_ += delta_th;  
	  nav_msgs::Odometry odom;

	  //since all odometry is 6DOF we'll need a quaternion created from yaw  
	  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th_);  
	  
	  //first, we'll publish the transform over tf  
	  geometry_msgs::TransformStamped odom_trans;  
	  odom_trans.header.stamp = current_time_;  
	  odom_trans.header.frame_id = "odom";  
	  odom_trans.child_frame_id = "base_link";  
	  
	  odom_trans.transform.translation.x = x_;  
	  odom_trans.transform.translation.y = y_;  
	  odom_trans.transform.translation.z = 0.0;  
	  odom_trans.transform.rotation = odom_quat;  
	  
	  //send the transform  
	  odom_broadcaster_.sendTransform(odom_trans); 

	  //next, we'll publish the odometry message over ROS   
	  odom.header.stamp = current_time_;  
	  odom.header.frame_id = "odom";  
	  
	  //set the position  
	  odom.pose.pose.position.x = x_;  
	  odom.pose.pose.position.y = y_;  
	  odom.pose.pose.position.z = 0.0;  
	  odom.pose.pose.orientation = odom_quat;  

	  //set the velocity
	  odom.child_frame_id = "base_link";
	  odom.twist.twist.linear.x = vx;
	  odom.twist.twist.linear.y = vy;
	  odom.twist.twist.angular.z = v_rad;

	  //publish the message
	  odom_pub.publish(odom);
	}

private:
	ros::Publisher odom_pub; 
	ros::Time last_time_;  
	tf::TransformBroadcaster odom_broadcaster_;  
	double x_ ;  
	double y_ ;  
	double th_ ;  
	double left_theta_to_speed;	//左转系数  
	double right_theta_to_speed;	//右转系数
	//double freq_slave; 		//freqency of basecontrol_slave
};
