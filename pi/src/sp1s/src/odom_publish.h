#pragma once
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include "util/vectorpairfile.h"
#include "gy85/hmc5883l.h"

/*
	 * cali_vel2angular.txt pair file contains map from linear velocity to angular velocity
	 * There are 10 ways to make turn.
	 * Refer to http://blog.csdn.net/crazyquhezheng/article/details/44279233
	 * Section 5, picture
	 * The turn order in this file is:
	 * 1,6,2,7,3,8,4,9,5,10
	 * The following macro define the order in the file
	 * which map the turning order in the refer web 
	 */
#define FORWARD_AROUND_LEFT_WHEEL 		0
#define BACKWARD_AROUND_LEFT_WHEEL 		1
#define FORWARD_AROUND_RIGHT_WHEEL		2
#define BACKWARD_AROUND_RIGHT_WHEEL 	3
#define FORWARD_LEFT_TURN		 		4
#define BACKWARD_LEFT_TURN		 		5
#define FORWARD_RIGHT_TURN		 		6
#define BACKWARD_RIGHT_TURN		 		7
#define SELF_ROTATE_CLOCKWISE	 		8
#define SELF_ROTATE_COUNTER_CLOCKWISE 	9

#define ANGULAR_ERR_TOLERANCE	0.1
class COdomPublish  
{  
public:  
  void Init(ros::NodeHandle& n, boost::shared_ptr<vectorpairfile> caliVel2Angular)
  {
	  x_ = 0.0;  
	  y_ = 0.0;  
	  th_ = 0.0;  
	         
	  left_theta_to_speed = 0.076;
	  right_theta_to_speed = 0.077;
	  freq_slave = 10;
	  last_time_ = ros::Time::now();
	  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);
	  m_vel2pwmPairfile = caliVel2Angular;

	  // calibrate compass
	  int x_offset=0,  y_offset=0,  z_offset;
	  n.param("compass_x_offset", x_offset, -163);
	  n.param("compass_y_offset", y_offset, -662);
	  n.param("compass_z_offset", z_offset, -49859);
	  m_compass.setOffset(x_offset, y_offset, z_offset);
	  th_ = -m_compass.read_bearing();
  }
  /*
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
  }*/
  void odom_to_speed(double cmd_twist_x, double cmd_twist_y, double cmd_twist_rotation,
  		  double& Lwheelspeed, double& Rwheelspeed)
  {
	  // Forget the Y velocity, it is always 0

	  // In the case to STOP!
	  if(DBLCMPEQ(cmd_twist_x, 0) && DBLCMPEQ(cmd_twist_rotation, 0))
	  {
		  Lwheelspeed = 0;
		  Rwheelspeed = 0;
		  return;
	  }
	  // In the case to go straight
	  else if(DBLCMPEQ(cmd_twist_rotation, 0))
	  {
		  Lwheelspeed = cmd_twist_x;
		  Rwheelspeed = cmd_twist_x;
		  return;
	  }
	  // In the case to self rotate
	  else if(DBLCMPEQ(cmd_twist_x, 0))
	  {
		  boost::shared_ptr<vectorPair> reverseWheel;
		  int left_dir, right_dir;
		  if(cmd_twist_rotation > 0)
		  {
			  reverseWheel = (*m_vel2pwmPairfile)[SELF_ROTATE_COUNTER_CLOCKWISE];
			  left_dir = -1;
			  right_dir = 1;
		  }
		  else
		  {
			  reverseWheel = (*m_vel2pwmPairfile)[SELF_ROTATE_CLOCKWISE];
			  left_dir = 1;
			  right_dir = -1;
		  }
		  double vel_diff = reverseWheel->Y2X(fabs(cmd_twist_rotation));
		  Lwheelspeed = vel_diff / 2 * left_dir;
		  Rwheelspeed = vel_diff / 2 * right_dir;
		  return;
	  }
	  boost::shared_ptr<vectorPair> singleWheel;
	  boost::shared_ptr<vectorPair> bothWheel;
	  boost::shared_ptr<vectorPair> reverseWheel;
	  bool bForward;
	  bool bLeftTurn;
	  if(DBLCMPGT(cmd_twist_x, 0))
	  {
		  bForward = true;
		  if(DBLCMPGT(cmd_twist_rotation, 0))
		  {
			  // Left forward turn
			  singleWheel 	= (*m_vel2pwmPairfile)[FORWARD_AROUND_LEFT_WHEEL];
			  bothWheel 	= (*m_vel2pwmPairfile)[FORWARD_LEFT_TURN];
			  reverseWheel 	= (*m_vel2pwmPairfile)[SELF_ROTATE_COUNTER_CLOCKWISE];

			  bLeftTurn = true;
		  }
		  else
		  {
			  // Right forward turn
			  singleWheel 	= (*m_vel2pwmPairfile)[FORWARD_AROUND_RIGHT_WHEEL];
			  bothWheel 	= (*m_vel2pwmPairfile)[FORWARD_RIGHT_TURN];
			  reverseWheel 	= (*m_vel2pwmPairfile)[SELF_ROTATE_CLOCKWISE];

			  bLeftTurn = false;
		  }
	  }
	  else
	  {
		  // Backward moving
		  bForward = false;
		  if(DBLCMPGT(cmd_twist_rotation, 0))
		  {
			  // Left backward turn
			  singleWheel 	= (*m_vel2pwmPairfile)[BACKWARD_AROUND_LEFT_WHEEL];
			  bothWheel 	= (*m_vel2pwmPairfile)[BACKWARD_LEFT_TURN];
			  reverseWheel 	= (*m_vel2pwmPairfile)[SELF_ROTATE_CLOCKWISE];
			  bLeftTurn = true;
		  }
		  else
		  {
			  // Right backward turn
			  singleWheel 	= (*m_vel2pwmPairfile)[BACKWARD_AROUND_RIGHT_WHEEL];
			  bothWheel 	= (*m_vel2pwmPairfile)[BACKWARD_RIGHT_TURN];
			  reverseWheel 	= (*m_vel2pwmPairfile)[SELF_ROTATE_COUNTER_CLOCKWISE];
			  bLeftTurn = false;
		  }
	  }
	  /*
	   * Suppose turn around a wheel by the required linear velocity
	   * could meet the angular velocity.
	   *
	   * If slower than the required  angular velocity, need to self rotate
	   * by turning two wheels in different direction.
	   *
	   * If faster than the required  angular velocity, need to turn two wheels
	   * in same direction.
	   */
	  double target_linear_vel = fabs(cmd_twist_x);
	  double target_angular_vel = fabs(cmd_twist_rotation);
	  double singleWheel_angular_Vel = singleWheel->X2Y(target_linear_vel*2);
	  double err = fabs(singleWheel_angular_Vel - target_angular_vel);
	  if(DBLCMPLE(err, ANGULAR_ERR_TOLERANCE))
	  {
		  // Turn around a wheel, only one wheel run
		  if(bLeftTurn)
		  {
			  Lwheelspeed = 0;
			  Rwheelspeed = cmd_twist_x * 2;
		  }
		  else
		  {
			  Lwheelspeed = cmd_twist_x * 2;
			  Rwheelspeed = 0;
		  }
		  return;
	  }
	  // Refer to page http://blog.csdn.net/crazyquhezheng/article/details/44279233
	  //   RightWheelV = RobotV + YawRate / delta_t / 2 ;
	  //   LeftWheelV = RobotV - YawRate / delta_t / 2 ;
	  //
	  // Here use delta_vel to replace YawRate / delta_t

	  double delta_vel = 0.0;
	  if(DBLCMPLT(target_angular_vel,singleWheel_angular_Vel))
	  {
		  // Both wheel go same direction but different velocity
		  delta_vel = bothWheel->InterpolateY2X(target_angular_vel);
		  if(!bLeftTurn)
			  delta_vel = -delta_vel;
	  }
	  else
	  {
		  // Self rotate, two wheels go different directions
		  delta_vel = cmd_twist_rotation / reverseWheel->m_dFactor;
	  }
	  Rwheelspeed = cmd_twist_x +  delta_vel / 2 ;
	  Lwheelspeed = cmd_twist_x -  delta_vel / 2 ;
  }
  /*
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
	    rad_z = delta_speed  * theta_to_speed * freq_slave;
	    vx = (vel_right + vel_left)/2.0;
	    vy = 0.0;
	}*/

  void speed_to_odom(float vel_left, float vel_right,
  		           float& vx, float& vy, float& yaw)
  {
	  vx = (vel_right + vel_left)/2.0;
	  vy = 0.0;
	  yaw = -m_compass.read_bearing();
  }

  void SpeedCallback(float speed_left, float speed_right)
  {
	  ros::Time current_time_ = ros::Time::now();  
	  float vx=0.0, vy=0.0, yaw=0.0;
	  speed_to_odom(speed_left,speed_right, vx, vy, yaw);

	  //compute odometry in a typical way given the velocities of the robot
	  double dt =  (current_time_ - last_time_).toSec();
	  last_time_ = current_time_;
	  double delta_x = (vx * cos(yaw) - vy * sin(yaw)) * dt;
	  double delta_y = (vx * sin(yaw) + vy * cos(yaw)) * dt;
	  double delta_th = yaw - th_;
	  double yaw_rate = delta_th / dt;
	  x_ += delta_x;  
	  y_ += delta_y;  
	  th_ = yaw;
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
	  odom.twist.twist.angular.z = yaw_rate;

	  //publish the message
	  odom_pub.publish(odom);
  }

private:
	ros::Publisher odom_pub; 
	ros::Time last_time_;  
	tf::TransformBroadcaster odom_broadcaster_;
	boost::shared_ptr<vectorpairfile> m_vel2pwmPairfile;
	hmc5883l m_compass;

	double x_ ;  
	double y_ ;  
	double th_ ;  
	double left_theta_to_speed;	//左转系数  
	double right_theta_to_speed;	//右转系数
	double freq_slave; 		//freqency of basecontrol_slave
};
