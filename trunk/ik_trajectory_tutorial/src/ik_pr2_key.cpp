#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <kinematics_msgs/IKQuery.h>
#include <kinematics_msgs/IKService.h>
#include <kinematics_msgs/FKService.h>
#include <ik_trajectory_tutorial/ExecuteCartesianIKTrajectory.h>
#include <vector>

#define KEYCODE_R (0x43)
#define KEYCODE_L (0x44)
#define KEYCODE_U (0x41)
#define KEYCODE_D (0x42)
#define KEYCODE_Q (0x71)

#define KEYCODE_P (0x70)
#define KEYCODE_N (0x6e)

#define INIT_POS_X ( 0.55)
#define INIT_POS_Y (-0.19)
#define INIT_POS_Z ( 0.70)
#define INIT_ORI_X ( 0.02)
#define INIT_ORI_Y (-0.09)
#define INIT_ORI_Z ( 0.0)
#define INIT_ORI_W ( 1.0)

#define DLENGTH (0.04)

class TeleopPr2
{
public:
  TeleopPr2();
  void keyLoop();
  void call_ik();
private:
  ros::NodeHandle nh_;
  double dx_, dy_, dz_;
  geometry_msgs::Pose pose_;
  ros::ServiceClient ik_client_;
};

#define IK_SERVICE_NAME "execute_cartesian_ik_trajectory"

TeleopPr2::TeleopPr2():
  nh_(),
  pose_(),
  ik_client_(nh_.serviceClient<ik_trajectory_tutorial::ExecuteCartesianIKTrajectory>(IK_SERVICE_NAME))
{
  dx_=dy_=dz_=0;

  pose_.position.x = INIT_POS_X;
  pose_.position.y = INIT_POS_Y;
  pose_.position.z = INIT_POS_Z;

  pose_.orientation.x = INIT_ORI_X;
  pose_.orientation.y = INIT_ORI_Y;
  pose_.orientation.z = INIT_ORI_Z;
  pose_.orientation.w = INIT_ORI_W;


  ROS_INFO("Waiting for services to be ready");
  ros::service::waitForService(IK_SERVICE_NAME);
  
  ROS_INFO("Services ready");
}

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "teleop_pr2");
  TeleopPr2 teleop_pr2;

  signal(SIGINT,quit);

  teleop_pr2.keyLoop();
  
  return(0);
}


void TeleopPr2::call_ik()
{
  ik_trajectory_tutorial::ExecuteCartesianIKTrajectory ik_req;
  ik_req.request.header = roslib::Header();
  ik_req.request.header.frame_id = "base_link";
  ik_req.request.header.stamp = ros::Time::now();

  //ik_req.poses = std::vector<geometry_msgs::Pose>;
  ik_req.request.poses.resize(0);
  ik_req.request.poses.push_back(pose_);
  ROS_INFO("Calling IK service");
  printf("q%f %f %f, %f %f %f %f\n",
	 pose_.position.x,
	 pose_.position.y,
	 pose_.position.z,
	 pose_.orientation.x,
	 pose_.orientation.y,
	 pose_.orientation.z,
	 pose_.orientation.w);
  ik_client_.call(ik_req);

  if (ik_req.response.success)
  {
    ROS_INFO("success!");
  }
  else
  {
    ROS_ERROR("fail!");
    //back to last pose
    pose_.position.x -= dx_;
    pose_.position.y -= dy_;
    pose_.position.z -= dz_;

  }
}

void TeleopPr2::keyLoop()
{
  char c;
  bool dirty=false;


  // get the console in raw mode                                                              
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file                         
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the right arm.");


  for(;;)
  {
    // get the next event from the keyboard  
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    dx_=dy_=dz_=0;

    printf("code=%x\n", c);

    switch(c)
    {
      case KEYCODE_L:
        ROS_DEBUG("LEFT");
        dy_ += DLENGTH;
        dirty = true;
        break;
      case KEYCODE_R:
        ROS_DEBUG("RIGHT");
        dy_ -= DLENGTH;
        dirty = true;
        break;
      case KEYCODE_U:
        ROS_DEBUG("UP");
        dx_ += DLENGTH;
        dirty = true;
        break;
      case KEYCODE_D:
        ROS_DEBUG("DOWN");
        dx_ -= DLENGTH;
        dirty = true;
        break;
      case KEYCODE_P:
        ROS_DEBUG("P");
        dz_ += DLENGTH;
        dirty = true;
        break;
      case KEYCODE_N:
        ROS_DEBUG("N");
        dz_ -= DLENGTH;
        dirty = true;
        break;
    }
   
    if(dirty ==true)
    {
      pose_.position.x += dx_;
      pose_.position.y += dy_;
      pose_.position.z += dz_;

      call_ik();
      dirty=false;
    }
  }

  return;
}



