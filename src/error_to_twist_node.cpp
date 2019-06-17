#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <error_to_twist/Errors.h>
#include <math.h>
#include <iostream>

#define W_NOM 0.5 //Normal angular velocity
#define MAX_ETF 0.026179939 //Maximum angylar error for rotate(+/- 1.5 graus)
#define HIST_ETF 0.034906585 //Maximum error hysteresis
#define LIN_VEL_NOM 0.9 //Normal forward velocity
#define GAIN_FWD 1.65 //Error correction angular speed
#define DIST_DA 1 //De-acceleration distance, in meters
#define LIN_VEL_DA 0.5 //Low linear velocity for preparation for stop
#define GAIN_DA 0.825 //De-acceleration angular correction

enum states {
	NO_MISSION,
	ROTATE,
	GO_FORWARD,
	DE_ACCEL,
} state;

enum events {
	STOP,
	START_MISSION,
	ROTATION,
	DE_ACCELERATION,
} event;

double DIST_ERROR = 0;
double ANGLE_ERROR = 0;

void errorsCallback(const error_to_twist::Errors::ConstPtr& msg) {
	DIST_ERROR = msg->distance_error;
	ANGLE_ERROR = msg->angle_error;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "error_to_twist_node");
	ros::NodeHandle n("~");
	ros::Rate loop_rate(20);

	geometry_msgs::Twist twist_msg; 
	twist_msg.linear.x = 0; twist_msg.angular.z = 0;
	ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
	ros::Subscriber sub_errors = n.subscribe("/errors", 10, errorsCallback);


	while(ros::ok()) {
		ros::spinOnce();
		std::cout << DIST_ERROR << " " <<  ANGLE_ERROR << std::endl;

		switch(state) {
			case NO_MISSION:
				if(DIST_ERROR != 0)
					if(fabs(ANGLE_ERROR) < MAX_ETF) event = START_MISSION;
					else event = ROTATION;

				switch(event) {
					case START_MISSION:
						state = GO_FORWARD;
					break;

					case ROTATION:
						state = ROTATE;
					break;

					default:
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;
				}
			break;

			case ROTATE:
				if(fabs(ANGLE_ERROR) < MAX_ETF) event = START_MISSION;
				if(ANGLE_ERROR == 0 && DIST_ERROR == 0) event = STOP;

				switch(event) {
					case START_MISSION:
						state = GO_FORWARD;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;

					case STOP:
						state = NO_MISSION;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;

					default:
						twist_msg.angular.z = ANGLE_ERROR < 0 ? -W_NOM : W_NOM;
						twist_msg.linear.x = 0;
					break;
				}
			break;


			case GO_FORWARD:
				if(DIST_ERROR < DIST_DA) event = DE_ACCELERATION;
				if(fabs(ANGLE_ERROR) > MAX_ETF + HIST_ETF) event = ROTATION;
				if(DIST_ERROR == 0 && ANGLE_ERROR == 0) event = STOP;

				switch(event) {
					case DE_ACCELERATION:
						state = DE_ACCEL;
					break;

					case ROTATION:
						state = ROTATE;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;

					case STOP:
						state = NO_MISSION;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;

					default:
						twist_msg.angular.z = GAIN_FWD * ANGLE_ERROR;
						twist_msg.linear.x = LIN_VEL_NOM;
					break;
				}
			break;

			case DE_ACCEL:
				if(DIST_ERROR == 0) event = STOP;

				switch(event) {
					case STOP:
						state = NO_MISSION;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;

					default:
						twist_msg.angular.z = GAIN_DA * ANGLE_ERROR;
						twist_msg.linear.x = LIN_VEL_DA;
					break;
				}
			break;
		}

		std::cout << state << std::endl;
		pub_twist.publish(twist_msg);
		loop_rate.sleep();
	}

	ros::shutdown();
}
