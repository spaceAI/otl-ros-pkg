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

//#include <pr2_gazebo_plugins/gazebo_ros_controller_manager.h>
#include <gazebo_ros_test.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <set>
#include <gazebo/Global.hh>
#include <gazebo/XMLConfig.hh>
#include <gazebo/Model.hh>
#include <gazebo/HingeJoint.hh>
#include <gazebo/SliderJoint.hh>
#include <gazebo/Simulator.hh>
#include <gazebo/gazebo.h>
#include <angles/angles.h>
#include <gazebo/GazeboError.hh>
#include <gazebo/ControllerFactory.hh>
#include <urdf/model.h>
#include <map>

#define JOINT_MAX_FORCE (1.0)

namespace gazebo {

GZ_REGISTER_DYNAMIC_CONTROLLER("gazebo_ros_test", GazeboRosTest);

GazeboRosTest::GazeboRosTest(Entity *parent)
  : Controller(parent),
    pgain_(2.0),
    dgain_(0.0)
{
  this->parent_model_ = dynamic_cast<Model*>(this->parent);

  if (!this->parent_model_)
    gzthrow("GazeboRosTest controller requires a Model as its parent");

  Param::Begin(&this->parameters);
  this->robotParamP = new ParamT<std::string>("robotParam", "robot_description", 0);
  this->robotNamespaceP = new ParamT<std::string>("robotNamespace", "/", 0);
  this->setModelsJointsStatesServiceNameP = new ParamT<std::string>("setModelsJointsStatesServiceName","set_models_joints_states", 0);
  Param::End();

}

/// \brief callback for setting models joints states


GazeboRosTest::~GazeboRosTest()
{
  delete this->robotParamP;
  delete this->robotNamespaceP;
  //  delete this->cm_; 
  delete this->rosnode_;
  delete this->ros_spinner_thread_;
}

/**
   angle-vectorを受け取ったときに行う行動
 */
void GazeboRosTest::cmdCallback(const std_msgs::Float64::ConstPtr& jointState)
{
  /* これでいいのかな？ */
  // vector copy
  for (std::vector<double>::iterator it = cmd_.begin(); it != cmd_.end(); it++)
    {
      *it = jointState->data;
      ROS_ERROR("setting %f", *it);
    }

  /* そのまま公開しちゃう */
  angles_pub_.publish(jointState);
  usleep(250*1000);
}

/* pgain setting */
void GazeboRosTest::cmdCallbackPgain(const std_msgs::Float64::ConstPtr& gain)
{
  pgain_ = gain->data;
}

/* dgain setting */
void GazeboRosTest::cmdCallbackDgain(const std_msgs::Float64::ConstPtr& gain)
{
  dgain_ = gain->data;
}

void GazeboRosTest::LoadChild(XMLConfigNode *node)
{
  // get parameter name
  this->robotParamP->Load(node);
  this->robotParam = this->robotParamP->GetValue();
  this->robotNamespaceP->Load(node);
  this->robotNamespace = this->robotNamespaceP->GetValue();

  int argc = 0;
  char** argv = NULL;

  // read pr2 urdf

  if (!this->urdf_model_.initFile("/home/ogutti/ros/pkgs/otl-ros-pkg/sandbox/gazebo_test/urdf/object.urdf"))
    {
      ROS_ERROR("load urdf error!");
    }

  std::vector <boost::shared_ptr<urdf::Link> > links;
  urdf_model_.getLinks(links);

  for (std::vector <boost::shared_ptr<urdf::Link> >::iterator it = links.begin(); it != links.end(); it++)
    {
      std::vector <boost::shared_ptr<urdf::Joint> > joints = (*it)->child_joints;
      for ( std::vector<boost::shared_ptr<urdf::Joint> >::iterator j = joints.begin(); j !=joints.end(); j++)
	{
	  ROS_ERROR("name=%s", (*j)->name.c_str());
	  gazebo::Joint *joint = this->parent_model_->GetJoint((*j)->name);
	  if (joint)
	    {
	      this->joints_.push_back(joint);
	      joint->SetParam(dParamVel, 0);
	      joint->SetParam(dParamFMax, JOINT_MAX_FORCE);
	    }
	  else
	    {
	      //ROS_WARN("A joint named \"%s\" is not part of Mechanism Controlled joints.\n", joint_name.c_str());
	      this->joints_.push_back(NULL);
	    }
	}
    }

  ros::init(argc,argv,"gazebo",ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  this->rosnode_ = new ros::NodeHandle(this->robotNamespace);  // namespace comes from gazebo_model spawner


  cmd_.resize(this->joints_.size());
  angles_pub_ = rosnode_->advertise<std_msgs::Float64>("iSOBOT/angles", 1);
  cmd_angles_sub_ = rosnode_->subscribe<std_msgs::Float64>("iSOBOT/cmd_angles", 1, &GazeboRosTest::cmdCallback, this);
  cmd_pgain_sub_ = rosnode_->subscribe<std_msgs::Float64>("iSOBOT/pgain", 1, &GazeboRosTest::cmdCallbackPgain, this);
  cmd_dgain_sub_ = rosnode_->subscribe<std_msgs::Float64>("iSOBOT/dgain", 1, &GazeboRosTest::cmdCallbackDgain, this);

  ROS_INFO("starting gazebo_ros_test plugin in ns: %s",this->robotNamespace.c_str());

  //  this->hw_.current_time_ = ros::Time(Simulator::Instance()->GetSimTime());
}

void GazeboRosTest::InitChild()
{
  this->ros_spinner_thread_ = new boost::thread( boost::bind( &ros::spin ) );
}

void GazeboRosTest::UpdateChild()
{
  // 状態アップデート
  // Copies the commands from the mechanism joints into the gazebo joints.
  ROS_ERROR("num=%d", this->joints_.size());
  for (unsigned int i = 0; i < this->joints_.size(); ++i)
  {
    if (!this->joints_[i])
      continue;

    //    double damping_coef = 100.0;

    switch (this->joints_[i]->GetType())
    {
    case Joint::HINGE: {
      // PD control
      HingeJoint *hj = (HingeJoint*)this->joints_[i];
      double current_velocity = hj->GetAngleRate();
      double current_angle = hj->GetAngle();
      double damping_force = dgain_ * (0 - current_velocity);
      double diff_force = pgain_ * (cmd_[i] - current_angle);
      double effort_command = diff_force + damping_force;
      ROS_ERROR("c=%f,cmd=%f, f=%f, v=%f", current_angle, cmd_[i], effort_command, current_velocity);
      //      hj->SetTorque(effort_command);
      hj->SetParam(dParamVel, effort_command);
      break;
    }
    default:
      abort();
    }
  }

}

void GazeboRosTest::FiniChild()
{
  ROS_DEBUG("Calling FiniChild in GazeboRosTest");

  for (unsigned int i=0; i<this->joints_.size(); i++){
    if (this->joints_[i]){
      delete this->joints_[i];
      this->joints_[i] = NULL;
    }
  }
  //  delete this->fake_state_;
  this->ros_spinner_thread_->join();
}


} // namespace gazebo
