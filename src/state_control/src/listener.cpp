#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

// Function to convert from quarternion to euler angle 
// https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles 
void toEulerAngle(double x, double y, double z, double w, double& roll, double& pitch, double& yaw)
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (w * x + y * z);
	double cosr = +1.0 - 2.0 * (x * x + y * y);
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (w * y - z * x);
	if (fabs(sinp) >= 1)
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		pitch = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (w * z + x * y);
	double cosy = +1.0 - 2.0 * (y * y + z * z);  
	yaw = atan2(siny, cosy);
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
geometry_msgs::Pose2D current_location; //<--get from odometry
geometry_msgs::Pose2D destination; //<-- place where to go

// callback that updates current location (from odometry)
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg){

	//update location
	current_location.x = msg->pose.pose.position.x;
	current_location.y = msg->pose.pose.position.y;
	
	//update angle
	double x, y, z;
	toEulerAngle(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w,
		x, y, z
	);
	
	//no rotation on x or y
	current_location.theta = z;
}

//callback when receiving new commands
void moveCallback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
	//movement setup
	if(state == STATE_IDLE){

		ROS_INFO("Command received. Moving from (%f, %f) to (%f, %f). IDLE -> ROT1."); //nothing to do
		destination = *msg;

		//proceed to ROT1 (rot1 calculation)
		state = STATE_ROT1_CALC;
		
	}else{
		ROS_INFO("Trying to move while from non-idle state. Ignoring."); //nothing to do
	}
}

//setup
int main(int argc, char **argv)
{
	//starts at position (0,0)
	current_location.x = 0;
	current_location.y = 0;
	current_location.theta = 0;

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
  
	while(ros::ok()){

		//In this state the robot is doing nothing and waits 
		//for some action to be sent through the topic. State
		//changes to STATE_ROT1 when some command arrives.
		if(state == STATE_IDLE){

			//nothing to do (stay idle). state change is done via callback

		//In this state the robot is rotating to align its face towards 
		//the destination point.
		}else if(state == STATE_ROT1_CALC){

			ROS_INFO("State: STATE_ROT1_CALC");
		
			//double delta_x = x_atual - x_destino;
			//double delta_y = y_atual - y_destino;
			//double rot1 = atan2(delta_x, delta_y) - theta;
			
			state = STATE_ROT1_EXEC;
			ROS_INFO("State: STATE_ROT1_EXEC");
		
		}else if(state == STATE_ROT1_EXEC){
	
			geometry_msgs::Twist m;
			m.angular.z = 1;
			
			pub.publish(m);
			
			//if(odom.angle == target_angle){
			//	ROS_INFO("State: STATE_MOVE");
			//	state = STATE_MOVE;				
			//}
			
			/*float roll = 1.5707, pitch = 0, yaw = 0.707;    
			Quaternionf q;
			q = AngleAxisf(roll, Vector3f::UnitX())
				* AngleAxisf(pitch, Vector3f::UnitY())
				* AngleAxisf(yaw, Vector3f::UnitZ());
			std::cout << "Quaternion" << std::endl << q.coeffs() << std::endl;*/
	
		//move until checkpoint is reached
		}else if (state == STATE_MOVE){ 


			state = STATE_ROT2_CALC;
		}else if (state == STATE_ROT2_CALC){


			state = STATE_ROT2_EXEC;
		//turn the bot until it gets to the original orientation (change angular, turn2)		
		}else if(state == STATE_ROT2_EXEC){

			ROS_INFO("State: STATE_ROT2");
			state = STATE_STOP;

		//turn the bot until it gets to the original orientation (change angular, turn2)		
		}else if(state == STATE_STOP){

			ROS_INFO("State: STATE_STOP");
			state = STATE_IDLE;
		}
		
		ros::spinOnce();
	}
  
	return 0;
}