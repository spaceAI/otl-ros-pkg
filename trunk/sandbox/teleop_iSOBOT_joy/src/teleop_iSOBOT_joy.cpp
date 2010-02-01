#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <joy/Joy.h>


#define JOY_HEAD (16)
#define JOY_LARM (10)
#define JOY_RARM (11)
#define JOY_LLEG (8)
#define JOY_RLEG (9)

#define JOY_SOFF (0)
#define JOY_SON (3)
#define JOY_ON (1)

class TeleopISOBOT
{
public:
    TeleopISOBOT();

private:
    virtual void joyCallback(const joy::Joy::ConstPtr& joy);
    virtual void jointCallback(const sensor_msgs::JointState::ConstPtr& js);
    int mode_;
    double rate_;
    sensor_msgs::JointState js_;
    ros::NodeHandle nh_;
    ros::Publisher angles_pub_;
    ros::Subscriber joy_sub_;
    ros::Subscriber angles_sub_;
};

TeleopISOBOT::TeleopISOBOT()
{
    mode_ = JOY_LARM;
    rate_ = 1.0;

    js_.position.resize(17);

    angles_pub_ = nh_.advertise<sensor_msgs::JointState>("iSOBOT/cmd_angles", 1);
    joy_sub_ = nh_.subscribe<joy::Joy>("joy", 10, &TeleopISOBOT::joyCallback, this);
    angles_sub_ = nh_.subscribe<sensor_msgs::JointState>("/iSOBOT/angles", 10, &TeleopISOBOT::jointCallback, this);
}

void TeleopISOBOT::jointCallback(const sensor_msgs::JointState::ConstPtr& js)
{
    // copy
    ROS_INFO("update joints");
    //js_.position = js->position;
}

void TeleopISOBOT::joyCallback(const joy::Joy::ConstPtr& joy)
{
//    turtlesim::Velocity vel;
//    vel.angular = a_scale_*joy->axes[angular_];
//    vel.linear = l_scale_*joy->axes[linear_];
//    vel_pub_.publish(vel);
    static int iterator = 0;
    std::cout << "IN!" << std::endl;

    if ( joy->buttons[JOY_HEAD] == JOY_ON)
    {
	mode_ = JOY_HEAD;
	iterator = 0;
    }
    else if ( joy->buttons[JOY_LARM] == JOY_ON)
    {
	mode_ = JOY_LARM;
	iterator = 1;
    }
    else if ( joy->buttons[JOY_RARM] == JOY_ON)
    {
	mode_ = JOY_RARM;
	iterator = 4;
    }
    else if ( joy->buttons[JOY_LLEG] == JOY_ON)
    {
	mode_ = JOY_LLEG;
	iterator = 7;
    }
    else if ( joy->buttons[JOY_RLEG] == JOY_ON)
    {
	mode_ = JOY_RLEG;
	iterator = 12;
    }
    
    if (mode_ == JOY_HEAD ||
	mode_ == JOY_LARM ||
	mode_ == JOY_RARM ||
	mode_ == JOY_LLEG ||
	mode_ == JOY_RLEG)
    {
	std::cout << "moving" << std::endl;
	std::cout << "size=" << js_.position.size() << std::endl;

	js_.position[iterator+0] += rate_ * joy->buttons[4];
	js_.position[iterator+0] -= rate_ * joy->buttons[6];
	
	js_.position[iterator+1] += rate_ * joy->buttons[5];
	js_.position[iterator+1] -= rate_ * joy->buttons[7];

	js_.position[iterator+2] += rate_ * joy->buttons[12];
	js_.position[iterator+2] -= rate_ * joy->buttons[14];
	
	js_.position[iterator+3] += rate_ * joy->buttons[15];
	js_.position[iterator+3] -= rate_ * joy->buttons[13];

	angles_pub_.publish(js_);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "teleop_iSOBOT");
    TeleopISOBOT teleop;
    ros::spinOnce();
    ros::spin();
}
