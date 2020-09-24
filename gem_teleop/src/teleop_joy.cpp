#include <ros/ros.h>
#include <ros/console.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/Joy.h>

class TeleopGEM {

	public:
		TeleopGEM();

	private:
		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
		ros::NodeHandle nh_;
		double accel_, angular_, brake_;
		double l_scale_, a_scale_;
		int start_, back_; // press both for enabling gamepad control
		int forward_, neutral_, reverse_;
		bool enabled_, forward_enabled_, neutral_enabled_, reverse_enabled_, forward_init_, reverse_init_, brake_init_;
		ros::Publisher vel_pub_;
		ros::Subscriber joy_sub_;
};

TeleopGEM::TeleopGEM() : accel_(5), brake_(2), angular_(0), start_(7), back_(6), forward_(0), neutral_(2), reverse_(1) {

	nh_.param("scale_angular", a_scale_, a_scale_);
	nh_.param("scale_linear", l_scale_, l_scale_);

	this->enabled_         = false;
	this->forward_enabled_ = false;
	this->reverse_enabled_ = false;
	this->neutral_enabled_ = false;
	this->forward_init_    = false;
	this->reverse_init_    = false;
	this->brake_init_      = false;


	// vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	vel_pub_ = nh_.advertise<ackermann_msgs::AckermannDrive>("ackermann_cmd", 10);
	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 100, &TeleopGEM::joyCallback, this);

}

void TeleopGEM::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) {

	// geometry_msgs::Twist twist;
	// twist.angular.z = a_scale_*joy->axes[angular_];
	// twist.linear.x = l_scale_*joy->axes[linear_];
	// twist.angular.z = a_scale_*joy->axes[angular_];
	// twist.linear.x = l_scale_*joy->axes[linear_];
	// vel_pub_.publish(twist);

	ackermann_msgs::AckermannDrive ackermann_drive;

	ackermann_drive.jerk                    = 0.0;
	ackermann_drive.acceleration            = 0.0;
	ackermann_drive.steering_angle_velocity = 0.0;
	ackermann_drive.steering_angle          = 0.0;
	ackermann_drive.speed                   = 0.0;

	if(enabled_ == false) {

		if((joy->buttons[start_]) && (joy->buttons[back_])) {
			// Enable gamepac control
			enabled_ = true;
		}

	} else {

		if(joy->buttons[back_]) {
			// Disable gamepac control
			enabled_ = false;
		}

	}


	if(enabled_) {

		if(joy->buttons[neutral_]) {
			neutral_enabled_ = true;
			forward_enabled_ = false;
			reverse_enabled_ = false;
			forward_init_    = false;
			reverse_init_    = false;
			brake_init_      = false;
		}

		if(joy->buttons[forward_]) {
			neutral_enabled_ = false;
			forward_enabled_ = true;
			reverse_enabled_ = false;
			forward_init_    = false;
			reverse_init_    = false;
			brake_init_      = false;
		}

		if(joy->buttons[reverse_]) {
			neutral_enabled_ = false;
			forward_enabled_ = false;
			reverse_enabled_ = true;
			forward_init_    = false;
			reverse_init_    = false;
			brake_init_      = false;
		}

		if (neutral_enabled_) {

			ackermann_drive.steering_angle = 0.0;
			ackermann_drive.speed = 0.0;

		} 

		if (forward_enabled_) {

			if ( (joy->axes[accel_] != 0) && (forward_init_ == false) ) {

				// Only enter once
				forward_init_ = true;

			} 

			if ( (joy->axes[brake_] != 0) && (brake_init_ == false) ) {

				// Only enter once
				brake_init_ = true;

			} 

			if (forward_init_) {

				if(joy->axes[accel_] != 1) {
					// send none-zero accel
					ackermann_drive.speed = l_scale_* (1-joy->axes[accel_]);
				}

			}

			if (brake_init_) {

				if (joy->axes[brake_] != 1) {
					if(joy->axes[brake_] == -1) {
						ackermann_drive.speed = 0.0;
					} else {
						ackermann_drive.speed = -(l_scale_/10.0) * (1-joy->axes[brake_]);
					}
						
				}

			}

		}
		
		if (reverse_enabled_) {

			if ( (joy->axes[accel_] != 0) && (forward_init_ == false) ) {

				// Only enter once
				reverse_init_ = true;

			} 

			if ( (joy->axes[brake_] != 0) && (brake_init_ == false) ) {

				// Only enter once
				brake_init_ = true;

			} 

			if (reverse_init_) {

				if(joy->axes[accel_] != 1) {
					// send none-zero accel
					ackermann_drive.speed = -l_scale_* (1-joy->axes[accel_]);
				}

			}

			if (brake_init_) {

				if (joy->axes[brake_] != 1) {
					if(joy->axes[brake_] == -1) {
						ackermann_drive.speed = 0.0;
					} else {
						ackermann_drive.speed = (l_scale_/10.0) * (1-joy->axes[brake_]);
					}
						
				}

			}

		}

		ackermann_drive.steering_angle = a_scale_*joy->axes[angular_];

		vel_pub_.publish(ackermann_drive);


	} else {
		neutral_enabled_ = false;
		forward_enabled_ = false;
		reverse_enabled_ = false;
		ackermann_drive.steering_angle = 0.0;
		ackermann_drive.speed = 0.0;
		vel_pub_.publish(ackermann_drive);
	}



}


int main(int argc, char** argv) {

	ros::init(argc, argv, "teleop_gem");

	TeleopGEM teleop_gem; 

	ros::spin();
}
















