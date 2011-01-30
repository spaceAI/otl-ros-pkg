#include <ros/ros.h>
#include "otl_gesture/cleaning_recog.h"

namespace otl_gesture
{
namespace cleaning
{
  Recognizer::Recognizer() {
  }

  Recognizer::~Recognizer() {
  }

  GestureType Recognizer::recog()
  {
    GestureType type = GESTURE_NONE;

    tf::StampedTransform transform_r;
    tf::StampedTransform transform_l;
    try{
      listener_.lookupTransform("/right_hand", "/torso", ros::Time(0), transform_r);
      listener_.lookupTransform("/left_hand",  "/torso", ros::Time(0), transform_l);
    }
    catch (tf::TransformException ex) {
      //ROS_ERROR("%s", ex.what());
      return GESTURE_NONE;
    }

    // ROS_INFO("x=%f, y=%f, z=%f",
    //  	     transform_r.getOrigin().x(),
    //  	     transform_r.getOrigin().y(),
    //  	     transform_r.getOrigin().z());
    if (transform_r.getOrigin().z() > 0.4 &&
	transform_l.getOrigin().z() > 0.4)
      {
	type = GESTURE_FORWARD;
      }
    else if (transform_r.getOrigin().z() > 0.2 &&
	     transform_l.getOrigin().z() > 0.2)
      {
	type = GESTURE_BACKWARD;
      }
    else if (transform_r.getOrigin().y() > 0.3 &&
	     transform_l.getOrigin().y() > 0.3)
      {
	type = GESTURE_ON;
      }
    else if (transform_l.getOrigin().x() > 0.3 &&
	     transform_r.getOrigin().x() < -0.3)
      {
	type = GESTURE_OFF;
      }
    
    else if (transform_l.getOrigin().x() > 0.3 &&
	     transform_r.getOrigin().x() > -0.2)
      {
	type = GESTURE_RIGHT;
      }
    else if (transform_l.getOrigin().x() < 0.2 &&
	     transform_r.getOrigin().x() < -0.3)
      {
	type = GESTURE_LEFT;
      }
    

    return type;
  }
  
}
}
