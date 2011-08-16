// -*- C++ -*-
/*!
 * @file  JoyRTM2velROS.cpp
 * @brief convert Joystick sample port to ROS twist msg
 * @date $Date$
 *
 * $Id$
 */

#include "JoyRTM2velROS.h"

// Module specification
// <rtc-template block="module_spec">
static const char* joyrtm2velros_spec[] =
  {
    "implementation_id", "JoyRTM2velROS",
    "type_name",         "JoyRTM2velROS",
    "description",       "convert Joystick sample port to ROS twist msg",
    "version",           "1.0.0",
    "vendor",            "OTL",
    "category",          "Sample",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    // Configuration variables
    "conf.default.max_linear_velocity", "0.2",
    "conf.default.max_angular_velocity", "0.4",
    // Widget
    "conf.__widget__.max_linear_velocity", "slider",
    "conf.__widget__.max_angular_velocity", "slider",
    // Constraints
    ""
  };
// </rtc-template>

/*!
 * @brief constructor
 * @param manager Maneger Object
 */
JoyRTM2velROS::JoyRTM2velROS(RTC::Manager* manager)
    // <rtc-template block="initializer">
  : RTC::DataFlowComponentBase(manager),
    m_joyInputIn("joy_input", m_joyInput)

    // </rtc-template>
    ,
    m_rosVelOut("cmd_vel", "rtm2ros", m_rosVel)
{
}

/*!
 * @brief destructor
 */
JoyRTM2velROS::~JoyRTM2velROS()
{
}



RTC::ReturnCode_t JoyRTM2velROS::onInitialize()
{
  // Registration: InPort/OutPort/Service
  // <rtc-template block="registration">
  // Set InPort buffers
  addInPort("joy_input", m_joyInputIn);
  
  // Set OutPort buffer
  
  // Set service provider to Ports
  
  // Set service consumers to Ports
  
  // Set CORBA Service Ports
  
  // </rtc-template>

  // <rtc-template block="bind_config">
  // Bind variables and configuration variable
  bindParameter("max_linear_velocity", m_max_linear_velocity, "0.2");
  bindParameter("max_angular_velocity", m_max_angular_velocity, "0.4");
  // </rtc-template>

  addRosOutPort("ros_velocity", m_rosVelOut);

  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t JoyRTM2velROS::onFinalize()
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onStartup(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onShutdown(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onActivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/


RTC::ReturnCode_t JoyRTM2velROS::onExecute(RTC::UniqueId ec_id)
{
  if(m_joyInputIn.isNew()) {
    m_joyInputIn.read();

    m_rosVel.linear.x = m_joyInput.data[1] * 0.005;
    m_rosVel.angular.z = -m_joyInput.data[0] * 0.010;

    if ( m_max_linear_velocity < m_rosVel.linear.x) {
      m_rosVel.linear.x = m_max_linear_velocity;
    } else if (-m_max_linear_velocity > m_rosVel.linear.x) {
      m_rosVel.linear.x = -m_max_linear_velocity;
    } else if (fabs(m_rosVel.linear.x) < 0.01) {
      m_rosVel.linear.x = 0.0;
    }
    if ( m_max_angular_velocity < m_rosVel.angular.z) {
      m_rosVel.angular.z = m_max_angular_velocity;
    } else if (-m_max_angular_velocity > m_rosVel.angular.z) {
      m_rosVel.angular.z = -m_max_angular_velocity;
    } else if (fabs(m_rosVel.angular.z) < 0.01) {
      m_rosVel.angular.z = 0.0;
    }

    m_rosVelOut.write();
  }
  return RTC::RTC_OK;
}

/*
RTC::ReturnCode_t JoyRTM2velROS::onAborting(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onError(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onReset(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onStateUpdate(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/

/*
RTC::ReturnCode_t JoyRTM2velROS::onRateChanged(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}
*/



extern "C"
{
 
  void JoyRTM2velROSInit(RTC::Manager* manager)
  {
    coil::Properties profile(joyrtm2velros_spec);
    manager->registerFactory(profile,
                             RTC::Create<JoyRTM2velROS>,
                             RTC::Delete<JoyRTM2velROS>);
  }
  
};


