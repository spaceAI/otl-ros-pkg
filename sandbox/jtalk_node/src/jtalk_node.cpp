/// Takashi Ogura <t.ogura@gmail.com>
/// BSD license
/// talk japanese node

#include <stdio.h>
#include "AquesTalk2.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

// talk speed
static int s_talk_speed = 100;

/// call back for /talk
void CommandCB(const std_msgs::String::ConstPtr& talk)
{
  int 	data_size;
  FILE *play_fd;

  // make wav data usign AquesTalk2
  unsigned char *wav_buff = AquesTalk2_Synthe_Roman(talk->data.c_str(), s_talk_speed, &data_size, NULL);

  if(wav_buff==0){
    ROS_ERROR("AquesTalk2_Synthe_Roman failed");
  }
  else
  {
    play_fd = popen("play -t wav -", "w");
    
    if (play_fd < 0)
    {
	ROS_ERROR("popen fail");
    }
    else
    {
      // write to play command stdin
      fwrite(wav_buff, 1, data_size, play_fd);
      // free the data buff
      AquesTalk2_FreeWave(wav_buff);
      pclose(play_fd);
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talk_node");
  ros::NodeHandle local_node("~");
  local_node.param<int>("talk_speed", s_talk_speed, 100);

  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("talk", 10, &CommandCB);
  ros::spin();
  return 0;
}
