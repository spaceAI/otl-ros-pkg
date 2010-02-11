/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef GAZEBO_TEST_H
#define GAZEBO_TEST_H

#include <vector>
#include <map>
#include <gazebo/Controller.hh>
#include <gazebo/Entity.hh>
#include <gazebo/Model.hh>
//#include "tinyxml/tinyxml.h"
#include <gazebo/Param.hh>
#include <ros/ros.h>
#include <urdf/model.h>

//#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64.h>

namespace gazebo
{
class HingeJoint;
class XMLConfigNode;

/// @addtogroup gazebo_dynamic_plugins Gazebo ROS Dynamic Plugins
/// @{
/** \defgroup gazebo_ros_controller_manager GazeboRosControllerMangager class

  \brief GazeboRosControllerMangager Plugin
  
  This is a controller plugin that provides interface between simulated robot and pr2_controller_manager.
  controller:gazebo_ros_controller_manager XML extension requires a model as its parent.  Please see pr2_description for
  example usages in the pr2_simulator.

  Example Usage:
  \verbatim
    <!-- GazeboRosControllerMangager -->
    <controller:gazebo_ros_controller_manager name="gazebo_ros_controller_manager" plugin="libgazebo_ros_controller_manager.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1000.0</updateRate>
      <robotParam>robot_description</robotParam>
      <robotNamespace>/</robotNamespace>
    </controller:gazebo_ros_controller_manager>
  \endverbatim
 
\{
*/

/**

  This is a controller plugin that provides interface between simulated robot and pr2_controller_manager.
  controller:gazebo_ros_controller_manager XML extension requires a model as its parent.  Please see pr2_description for
  example usages in the pr2_simulator.

  Gazebo simulator provides joint force/torque control for simulated joints and links.
  This plugin exposes a set of pseudo-actuator states to pr2_controller_manager through ros by
  the use of inverse transmissions as defined in pr2_mechanism_controllers.
  
   - Example Usage:
  \verbatim
    <!-- GazeboRosControllerMangager -->
    <controller:gazebo_ros_controller_manager name="gazebo_ros_controller_manager" plugin="libgazebo_ros_controller_manager.so">
      <alwaysOn>true</alwaysOn>
      <updateRate>1000.0</updateRate>
      <robotParam>robot_description</robotParam>
      <robotNamespace>/</robotNamespace>
    </controller:gazebo_ros_controller_manager>
  \endverbatim
   .
**/


class GazeboRosTest : public gazebo::Controller
{
public:
  GazeboRosTest(Entity *parent);
  virtual ~GazeboRosTest();

protected:
  // Inherited from gazebo::Controller
  virtual void LoadChild(XMLConfigNode *node);
  virtual void InitChild();
  virtual void UpdateChild();
  virtual void FiniChild();

private:

  Model *parent_model_;
  urdf::Model urdf_model_;

  ros::Publisher angles_pub_;
  ros::Subscriber cmd_angles_sub_;
  ros::Subscriber cmd_pgain_sub_;
  ros::Subscriber cmd_dgain_sub_;
  float pgain_;
  float dgain_;
  //  pr2_hardware_interface::HardwareInterface hw_;
  //  pr2_controller_manager::ControllerManager *cm_;
  void cmdCallback(const std_msgs::Float64::ConstPtr& jointState);
  void cmdCallbackPgain(const std_msgs::Float64::ConstPtr& gain);
  void cmdCallbackDgain(const std_msgs::Float64::ConstPtr& gain);
  /// @todo The fake state helps Gazebo run the transmissions backwards, so
  ///       that it can figure out what its joints should do based on the
  ///       actuator values.
  //pr2_mechanism_model::RobotState *fake_state_;
  std::vector<gazebo::Joint*>  joints_;
  std::vector<double>  cmd_;

  /// \brief Service Call Name
  private: ParamT<std::string> *setModelsJointsStatesServiceNameP;
  private: std::string setModelsJointsStatesServiceName;

  /*
   *  \brief pointer to ros node
   */
  ros::NodeHandle* rosnode_;

  /// \brief ros service
  private: ros::ServiceServer setModelsJointsStatesService;

  ///\brief ros service callback
  /*
   *  \brief tmp vars for performance checking
   */
  double wall_start, sim_start;

  /// \brief set topic name of robot description parameter
  ParamT<std::string> *robotParamP;
  ParamT<std::string> *robotNamespaceP;
  std::string robotParam;
  std::string robotNamespace;

  private: boost::thread* ros_spinner_thread_;
};

/** \} */
/// @}

}

#endif

