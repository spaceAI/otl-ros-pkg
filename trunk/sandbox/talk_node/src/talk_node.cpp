/**********************************************************************
	SampleTalk.c - AquesTalk2 Linux 規則音声合成 サンプルプログラム

	標準入力から音声記号列を１行読み込み、
	標準出力に音声波形(.wavフォーマット）を出力

	COPYRIGHT (C) 2010 AQUEST CORP.

	使用方法は、readme.txt を参照ください。
	
	2010/01/23	N.Yamazaki	Creation
**********************************************************************/
#include <stdio.h>
#include "talk_node/AquesTalk2.h"
#include <ros/ros.h>
#include <std_msgs/String.h>

void CommandCB(const std_msgs::String::ConstPtr& talk)
{
  int 	size;
  FILE *play;

  // 音声合成
  unsigned char *wav = AquesTalk2_Synthe_Roman(talk->data.c_str(), 100, &size, NULL);

  if(wav==0){
    ROS_ERROR("ERR:%d",size);
  }
  
  // 音声データ(wavフォーマット)の出力
  //fwrite(wav, 1, size, stdout);
  play = popen("/usr/bin/play -t wav -", "w");

  if (play < 0)
    {
      ROS_ERROR("popen fail");
    }
  fwrite(wav, 1, size, play);
  
  // 音声データバッファの開放
  AquesTalk2_FreeWave(wav);
  pclose(play);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talk_node");
  ros::NodeHandle node;
  ros::Subscriber sub = node.subscribe("talk", 10, &CommandCB);
  ros::spin();
  return 0;
}
