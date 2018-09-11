#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

//state definiton
#define STATE_IDLE 0 /* when no movement is occuring */
#define STATE_TUR1 1 /* when turning for the first time */
#define STATE_TUR2 2 /* when turning for the second time */
#define STATE_MOVE 3 /* forward movement to the checkpoint */

//state variable (prevents stupid people from sending commands during movement
char __state = STATE_IDLE;


//publisher ptr (needs to be outside since ros::init must come first)
ros::Publisher* pub_ptr;

//callback when receiving new commands
void gotoCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	//message to achieve the goal
	geometry_msgs::Twist m;
	
	ROS_INFO("Received command: [%d]", msg->linear.x);
	switch(__state){

		//do nothing, stay idle until some location is sent by host
		case STATE_IDLE:
			//TODO: missing condition to change state
			break;
		
		//turn the bot until it is facing the checkpoint (change angular, turn1)
		case STATE_TUR1:
			pub_ptr->publish(msg);
			//TODO: missing condition to change state
			break;
		
		//move until checkpoint is reached
		case STATE_MOVE: 

			m.linear.x = 0;
			m.linear.y = 0;
			m.linear.z = 0;
			m.angular.x = 0;
			m.angular.y = 0;
			m.angular.z = 0;
			
			//TODO: missing condition to change state
			pub_ptr->publish(m);
			ROS_INFO("Waiting for movement to finish");
			break;
		
		//turn the bot until it gets to the original orientation (change angular, turn2)
		case STATE_TUR2:
			pub_ptr->publish(msg);
			//TODO: missing condition to change state
			break;
	}
}

//setup
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber sub = n.subscribe("move", 1000, gotoCallback);;

  pub_ptr = &pub;
  __state = STATE_IDLE;

  ros::spin();

  return 0;
}
// %EndTag(FULLTEXT)%
