#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <ros/ros.h>
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include <kinematics_msgs/IKQuery.h>
#include <kinematics_msgs/IKService.h>
#include <kinematics_msgs/FKService.h>
#include "/home/ogutti/ros/pkgs/wg-ros-pkg-trunk/sandbox/joint_states_listener/srv/cpp/joint_states_listener/ReturnJointStates.h"
#include <ik_trajectory_tutorial/ExecuteCartesianIKTrajectory.h>
#include <vector>

#define MAX_JOINT_VEL 0.5  //in radians/sec

static const std::string ARM_IK_NAME = "/pr2_ik_right_arm/ik_service";
typedef actionlib::SimpleActionClient<pr2_controllers_msgs::
                                      JointTrajectoryAction> TrajClient;

class IKTrajectoryExecutor{

private:
  ros::NodeHandle node;
  ros::ServiceClient ik_client;
  ros::ServiceClient joint_states_client;
  ros::ServiceServer service;
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  kinematics_msgs::IKService::Request  ik_request;
  kinematics_msgs::IKService::Response ik_response;  
  TrajClient *action_client;

public:
  IKTrajectoryExecutor(){

    //create a client function for the IK service
    ik_client = node.serviceClient<kinematics_msgs::IKService>
      (ARM_IK_NAME, true);    

    //tell the joint trajectory action client that we want 
    //to spin a thread by default
    action_client = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    //wait for the action server to come up
    while(!action_client->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action action server to come up");
    }

    //use the return_joint_states service to get the current joint angles
    joint_states_client = node.serviceClient
      <joint_states_listener::ReturnJointStates>("return_joint_states");

    //wait for the return_joint_states service to be ready
    ROS_INFO("Waiting for return_joint_states service to be ready\n");
    ros::service::waitForService("return_joint_states");

    //register a service to input desired Cartesian trajectories
    service = node.advertiseService("execute_cartesian_ik_trajectory", 
        &IKTrajectoryExecutor::execute_cartesian_ik_trajectory, this);

    //have to specify the order of the joints we're sending in our 
    //joint trajectory goal, even if they're already on the param server
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

  }

  ~IKTrajectoryExecutor(){
    delete action_client;
  }


  //run inverse kinematics on a PoseStamped (7-dof pose 
  //(position + quaternion orientation) + header specifying the 
  //frame of the pose)
  //tries to stay close to double positions[7]
  //returns the solution angles in double solution[7] 
  bool run_ik(geometry_msgs::PoseStamped pose, double positions[7], 
              double solution[7]){

    kinematics_msgs::IKService::Request  ik_request;
    kinematics_msgs::IKService::Response ik_response;  
 
    ik_request.data.joint_names.push_back("r_shoulder_pan_joint");
    ik_request.data.joint_names.push_back("r_shoulder_lift_joint");
    ik_request.data.joint_names.push_back("r_upper_arm_roll_joint");
    ik_request.data.joint_names.push_back("r_elbow_flex_joint");
    ik_request.data.joint_names.push_back("r_forearm_roll_joint");
    ik_request.data.joint_names.push_back("r_wrist_flex_joint");
    ik_request.data.joint_names.push_back("r_wrist_roll_joint");

    ik_request.data.link_name = "r_wrist_roll_link"; 

    ik_request.data.pose_stamped = pose;
    ik_request.data.positions.resize(7);
    for(int i=0; i<7; i++) ik_request.data.positions[i] = positions[i];

    bool ik_service_call = ik_client.call(ik_request,ik_response);
    if(!ik_service_call){
      ROS_ERROR("IK service call failed!");  
      return 0;
    }
    for(int i=0; i<7; i++){
      solution[i] = ik_response.solution[i];
    }
    ROS_INFO("solution angles: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f", 
             solution[0],solution[1], solution[2],solution[3],
             solution[4],solution[5],solution[6]);
    ROS_INFO("IK service call succeeded");
    return 1;
  }


  //figure out where the arm is now  
  bool get_current_joint_angles(double current_angles[7]){
    int i;
    joint_states_listener::ReturnJointStates joint_states_srv;
    joint_states_srv.request.name.push_back("r_shoulder_pan_joint");
    joint_states_srv.request.name.push_back("r_shoulder_lift_joint");
    joint_states_srv.request.name.push_back("r_upper_arm_roll_joint");
    joint_states_srv.request.name.push_back("r_elbow_flex_joint");
    joint_states_srv.request.name.push_back("r_forearm_roll_joint");
    joint_states_srv.request.name.push_back("r_wrist_flex_joint");
    joint_states_srv.request.name.push_back("r_wrist_roll_joint");
    if(joint_states_client.call(joint_states_srv)){
      for(i=0; i<7; i++) current_angles[i] = 
                           joint_states_srv.response.position[i];
      return 1;
    }
    else{
      ROS_ERROR("Failed to call service joint_states_srv");
      return 0;
    }
  }

  //send a desired joint trajectory to the joint trajectory executor 
  //and wait for it to finish
  bool execute_joint_trajectory(std::vector<double *> joint_trajectory){
    int i, j; 
    int trajectorylength = joint_trajectory.size();

    //get the current joint angles
    double current_angles[7];    
    get_current_joint_angles(current_angles);

    //fill the goal message with the desired joint trajectory
    goal.trajectory.points.resize(trajectorylength+1);

    //set the first trajectory point to the current position
    goal.trajectory.points[0].positions.resize(7);
    goal.trajectory.points[0].velocities.resize(7);
    for(j=0; j<7; j++){
      goal.trajectory.points[0].positions[j] = current_angles[j];
      goal.trajectory.points[0].velocities[j] = 0.0; 
    }

    //make the first trajectory point start 0.5 seconds from when we run
    goal.trajectory.points[0].time_from_start = ros::Duration(0.5);     

    //fill in the rest of the trajectory
    double time_from_start = 0.;
    for(i=0; i<trajectorylength; i++){
      printf("goal point %d: ", i);
      goal.trajectory.points[i+1].positions.resize(7);
      goal.trajectory.points[i+1].velocities.resize(7);

      //fill in the joint positions (velocities of 0 mean that the arm
      //will try to stop briefly at each waypoint)
      for(j=0; j<7; j++){
        printf("%0.3f ", joint_trajectory[i][j]);
        goal.trajectory.points[i+1].positions[j] = joint_trajectory[i][j];
        goal.trajectory.points[i+1].velocities[j] = 0.0;
      }
      printf("\n");

      //compute a desired time for this trajectory point using a max 
      //joint velocity
      double max_joint_move = 0;
      for(j=0; j<7; j++){
        double joint_move = fabs(goal.trajectory.points[i+1].positions[j] 
                                 - goal.trajectory.points[i].positions[j]);
        if(joint_move > max_joint_move) max_joint_move = joint_move;
      }
      double seconds = max_joint_move/MAX_JOINT_VEL;
      ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
      time_from_start += seconds;
      goal.trajectory.points[i+1].time_from_start = 
        ros::Duration(time_from_start);
      printf("\n");
    }

    //when to start the trajectory
    goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(0.5);

    ROS_INFO("Sending goal to joint_trajectory_action");
    action_client->sendGoal(goal);

    action_client->waitForResult();

    //get the current joint angles for debugging
    get_current_joint_angles(current_angles);
    ROS_INFO("joint angles after trajectory: %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f %0.3f\n", current_angles[0],current_angles[1], current_angles[2],current_angles[3],current_angles[4],current_angles[5],current_angles[6]);

    if(action_client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Hooray, the arm finished the trajectory!");
      return 1;
    }
    ROS_INFO("The arm failed to execute the trajectory.");
    return 0;
  }


  //service function for execute_cartesian_ik_trajectory
  bool execute_cartesian_ik_trajectory(
         ik_trajectory_tutorial::ExecuteCartesianIKTrajectory::Request &req,
         ik_trajectory_tutorial::ExecuteCartesianIKTrajectory::Response &res){

    int trajectory_length = req.poses.size();
    int i, j;
    
    //IK takes in Cartesian poses stamped with the frame they belong to
    geometry_msgs::PoseStamped stamped_pose;
    stamped_pose.header = req.header;
    stamped_pose.header.stamp = ros::Time::now();
    bool success;
    std::vector<double *> joint_trajectory;

    //get the current joint angles (to find ik solutions close to)
    double last_angles[7];    
    get_current_joint_angles(last_angles);

    //find IK solutions for each point along the trajectory 
    //and stick them into joint_trajectory
    for(i=0; i<trajectory_length; i++){
      
      stamped_pose.pose = req.poses[i];
      printf("q[%d]:%f %f %f, %f %f %f %f\n",i,
	     req.poses[i].position.x,
	     req.poses[i].position.y,
	     req.poses[i].position.z,
	     req.poses[i].orientation.x,
	     req.poses[i].orientation.y,
	     req.poses[i].orientation.z,
	     req.poses[i].orientation.w);

	     
      double *trajectory_point = new double[7];
      success = run_ik(stamped_pose, last_angles, trajectory_point);
      joint_trajectory.push_back(trajectory_point);

      if(!success){
        ROS_ERROR("IK solution not found for trajectory point number %d!\n", i);
        return 0;
      }
      for(j=0; j<7; j++) last_angles[j] = trajectory_point[j];
    }        

    //run the resulting joint trajectory
    ROS_INFO("executing joint trajectory");
    success = execute_joint_trajectory(joint_trajectory);
    res.success = success;
    
    return success;
  }

};


int main(int argc, char** argv){

  //init ROS node
  ros::init(argc, argv, "cartesian_ik_trajectory_executor");
  //sleep(1);

  IKTrajectoryExecutor ik_traj_exec = IKTrajectoryExecutor();

  ROS_INFO("Waiting for cartesian trajectories to execute");
  ros::spin();
  
  return 0;
}
