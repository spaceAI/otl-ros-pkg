#include "otl_gesture/cleaning_recog.h"
#include <std_msgs/String.h>

int main(int argc, char** argv)
{
  using namespace otl_gesture::cleaning;

  ros::init(argc, argv, "clearning_recog_node");
  
  ros::NodeHandle node;
  Recognizer recog;
  ros::Publisher pub = node.advertise<std_msgs::String>("gesture", 1);
  std_msgs::String msg;

  ros::Rate rate(10.0);
  while(node.ok()) {
    GestureType type = recog.recog();
    switch (type)
      {
      case GESTURE_FORWARD:
	ROS_INFO("FORWARD");
	msg.data = "forward";
	pub.publish(msg);
	break;
      case GESTURE_BACKWARD:
	ROS_INFO("BACKWARD");
	msg.data = "backward";
	pub.publish(msg);
	break;
      case GESTURE_RIGHT:
	ROS_INFO("RIGHT");
	msg.data = "right";
	pub.publish(msg);
	break;
      case GESTURE_LEFT:
	ROS_INFO("LEFT");
	msg.data = "left";
	pub.publish(msg);
	break;
      case GESTURE_ON:
	msg.data = "on";
	pub.publish(msg);
	ROS_INFO("ON");
	break;
      case GESTURE_OFF:
	ROS_INFO("OFF");
	msg.data = "off";
	pub.publish(msg);
	break;
      default:
	break;
      }
  }
  
  return 0;
}
