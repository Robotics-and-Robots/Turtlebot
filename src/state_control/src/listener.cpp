#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

// 0,01%
#define ALLOWED_ERROR 0.0001

#define MIN_TURN_VEL 0.1
#define MIN_MOVE_VEL 0.1

// Function to convert from quarternion to euler angle 
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
void toEulerAngle(double x, double y, double z, double w, double* yaw){
	double siny = +2.0 * (w * z + x * y);
	double cosy = +1.0 - 2.0 * (y * y + z * z);  
	*yaw = atan2(siny, cosy);
}

//print pos info on console
void printPose(geometry_msgs::Pose2D pos){
	ROS_INFO("(%f, %f; %f)", pos.x, pos.y, pos.theta);
}

// Enum that model states for the robot movement
enum State{
	STATE_IDLE      = 0, // when no movement is occuring
	STATE_ROT1_CALC = 1, // calculation of target rot1 angle
	STATE_ROT1_EXEC = 2, // execution of rot1
	STATE_MOVE      = 3, // linear movement
	STATE_ROT2_CALC = 4, // calculation of target rot2 angle
	STATE_ROT2_EXEC = 5, // execution of rot2
	STATE_STOP      = 6, // cleanup 
};

//state variable (prevents stupid people from sending commands during movement)
State state = STATE_IDLE;

//temporaries
double rot1, rot2, trans;
geometry_msgs::Pose2D current_pose; //<--get from odometry
geometry_msgs::Pose2D destination; //<-- place where to go

// callback that updates current location (from odometry)
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

	//update location
	current_pose.x = msg->pose.pose.position.x;
	current_pose.y = msg->pose.pose.position.y;
	
	//update angle
	double z;
	toEulerAngle(
		msg->pose.pose.orientation.x, 
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w,
		&current_pose.theta
	);
}

//callback when receiving new commands
void moveCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//movement setup
	if(state != STATE_IDLE){
		ROS_INFO("Starting new movement without finishing previous.");
	}

	destination = *msg;

	ROS_INFO("STATE_ROT1_CALC");
	ROS_INFO("from: ");
	printPose(current_pose);
	ROS_INFO("to: ");
	printPose(destination);

	//proceed to rot1 calculation
	state = STATE_ROT1_CALC;
}

//state machine
void stateSpin(ros::Publisher* p){

	ros::Publisher pub = *p;

	//In this state the robot waits for some move to be sent 
	//through the "move" topic. State changes to STATE_ROT1 
	//when some command arrives.
	if(state == STATE_IDLE){

		//nothing to do (stay idle). state change is done via callback

	//In this state the robot is rotating to align its face towards 
	//the destination point.
	}else if(state == STATE_ROT1_CALC){

		ROS_INFO("State: STATE_ROT1_CALC");
	
		double delta_x = destination.x - current_pose.x;
		double delta_y = destination.y - current_pose.y;
		rot1 = atan2(delta_y, delta_x) - current_pose.theta;
		
		state = STATE_ROT1_EXEC;
		ROS_INFO("rot1 is %f", rot1);
		ROS_INFO("State: STATE_ROT1_EXEC");
	
	}else if(state == STATE_ROT1_EXEC){

		//turns until reaches rot1
		double err = (current_pose.theta - rot1);
		ROS_INFO("rotation err: %f", err);
		
		if(abs(err) > ALLOWED_ERROR){ 

			geometry_msgs::Twist m;
			m.angular.z = (abs(MIN_TURN_VEL) > abs(err)) ? MIN_TURN_VEL : err;
			m.angular.z = (err > 0) ? -abs(m.angular.z) : abs(m.angular.z);
			pub.publish(m);
											
		//when in rot1, stop turning and start movement
		}else{
			geometry_msgs::Twist m;
			m.angular.z = 0;
			pub.publish(m);
			
			ROS_INFO("State: STATE_MOVE");
			state = STATE_MOVE;
		}

	//move until checkpoint is reached
	}else if (state == STATE_MOVE){ 

		double err_x = current_pose.x - destination.x;
		double err_y = current_pose.y - destination.y;
		ROS_INFO("x_err: %f y_err: %f", err_x, err_y);
		
		//y?
		if(abs(err_x) > ALLOWED_ERROR){

			geometry_msgs::Twist m;
			m.linear.x = (abs(MIN_MOVE_VEL) > abs(err_x)) ? MIN_MOVE_VEL : err_x;
			m.linear.x = (err_x > 0) ? -abs(m.linear.x) : abs(m.linear.x);			
			pub.publish(m);	//move forward
			
		}else{
			geometry_msgs::Twist m;
			m.linear.x = 0;
			pub.publish(m);
			
			ROS_INFO("State: STATE_ROT2_CALC");
			state = STATE_ROT2_CALC;
		}

	}else if (state == STATE_ROT2_CALC){

		double delta_x = destination.x - current_pose.x;
		double delta_y = destination.y - current_pose.y;
		rot2 = destination.theta - current_pose.theta - rot1;
		
		state = STATE_ROT2_EXEC;
		ROS_INFO("rot2 is %f", rot2);
		ROS_INFO("State: STATE_ROT2_EXEC");

	//turn the bot until it gets to the original orientation (change angular, turn2)		
	}else if(state == STATE_ROT2_EXEC){

		//turns until reaches rot1
		//double err = (current_pose.theta - rot2);
		double err = current_pose.theta - destination.theta;
		ROS_INFO("rotation err: %f", err);
		
		if(abs(err) > ALLOWED_ERROR){ 

			geometry_msgs::Twist m;
			m.angular.z = (abs(MIN_TURN_VEL) > abs(err)) ? MIN_TURN_VEL : err;
			m.angular.z = (err > 0) ? -abs(m.angular.z) : abs(m.angular.z);
			pub.publish(m);
											
		//when in rot1, stop turning and start movement
		}else{
			geometry_msgs::Twist m;
			m.angular.z = 0;
			pub.publish(m);
			
			ROS_INFO("State: STATE_STOP");
			state = STATE_STOP;
		}
		
	//turn the bot until it gets to the original orientation (change angular, turn2)		
	}else if(state == STATE_STOP){

		ROS_INFO("State: STATE_IDLE");
		state = STATE_IDLE;
	}
}

//setup
int main(int argc, char **argv)
{
	//ros stuff
	ros::init(argc, argv, "listener");

	//subscribe to command interface and advertise into cmd_vel so
	//commands can be issued to the simulation
	ros::NodeHandle n;
	ros::Publisher pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	//data from other nodes 
	ros::Subscriber subMove = n.subscribe("move", 1000, moveCallback);
	ros::Subscriber subOdom = n.subscribe("odom", 1000, odomCallback);

	//starts in idle position (no movement being done)
	state = STATE_IDLE;
	ROS_INFO("STATE_IDLE");
  
	do{
		ros::spinOnce();
		stateSpin(&pub);
	}while(ros::ok());
  
	return 0;
}