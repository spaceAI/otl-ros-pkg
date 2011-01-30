#ifndef OTL_GESTURE_CLEANING_RECOG
#define OTL_GESTURE_CLEANING_RECOG

#include <tf/transform_listener.h>

namespace otl_gesture
{
namespace cleaning
{
  enum GestureType {GESTURE_NONE, GESTURE_FORWARD, GESTURE_BACKWARD, GESTURE_RIGHT, GESTURE_LEFT, GESTURE_OFF, GESTURE_ON};
  class Recognizer
  {
  public:
    Recognizer();
    virtual ~Recognizer();
    GestureType recog();
  private:
    tf::TransformListener listener_;
  };
}
}

#endif //OTL_GESTURE_CLEANING_RECOG
