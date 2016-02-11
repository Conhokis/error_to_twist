#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <error_to_twist/Errors.h>
#include <math.h>
#include <iostream>

#define W_NOM 0.5 //Normal angular velocity
#define W_NOM_DE 0.075 //De-accelerate from rotation velocity
#define MAX_ETF 0.0025 //Maximum angylar error for rotate(+/- 1.5 graus)
#define HIST_ETF 0.05 //Maximum error hysteresis
#define LIN_VEL_NOM 0.5 //Normal forward velocity
#define GAIN_FWD 4.25//Error correction angular speed
#define DIST_DA 1.5 //De-acceleration distance, in meters
#define DIST_FINAL_DA 0.5 //Final deaceleration distance
#define LIN_VEL_DA 0.3 //Low linear velocity for preparation for stop
#define GAIN_DA 2.5 //De-acceleration angular correction

enum states {
	NO_MISSION,
	ROTATE,
	GO_FORWARD,
	DE_ACCEL,
	FINAL_DE_ACCEL,
	DE_ROTATE,
} state;

enum events {
	STOP,
	START_MISSION,
	ROTATION,
	DE_ACCELERATION,
	FINAL_DE,
	DE_ROTATION,
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
	ros::Publisher pub_twist = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
	ros::Subscriber sub_errors = n.subscribe("/errors", 1, errorsCallback);


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
				if(fabs(ANGLE_ERROR) < 3 * (MAX_ETF + HIST_ETF)) event = DE_ROTATION;
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

					case DE_ROTATION:
						state = DE_ROTATE;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;

					default:
						twist_msg.angular.z = ANGLE_ERROR < 0 ? -W_NOM : W_NOM;
						twist_msg.linear.x = 0;
					break;
				}
			break;

			case DE_ROTATE:
				if(fabs(ANGLE_ERROR) < MAX_ETF) event = START_MISSION;
		
				switch(event) {
					case START_MISSION:
						state = GO_FORWARD;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;				
					break;				
					
					default:
						twist_msg.angular.z = ANGLE_ERROR < 0 ? -W_NOM_DE : W_NOM_DE;
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
				if(DIST_ERROR < DIST_FINAL_DA) event = FINAL_DE;

				switch(event) {
					case STOP:
						state = NO_MISSION;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;
					
					case FINAL_DE:
						state = FINAL_DE_ACCEL;
					break;

					default:
						twist_msg.angular.z = GAIN_DA * ANGLE_ERROR;
						twist_msg.linear.x = LIN_VEL_DA;
					break;
				}
			break;

			case FINAL_DE_ACCEL:
				if(DIST_ERROR == 0) event = STOP;

				switch(event) {
					case STOP:
						state = NO_MISSION;
						twist_msg.angular.z = 0;
						twist_msg.linear.x = 0;
					break;

					default:
						twist_msg.angular.z = 0;
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
