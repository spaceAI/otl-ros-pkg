#include <lv1_arm/lv1.h>
#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

class Lv1Node
{
public:
    Lv1Node();
    void Ini();
    void Motion(const std_msgs::String::ConstPtr &motion);
    void PlaySosogi(int id);
private:
    Lv1Interface lv1_;
    uint32_t tray_id_;
    ros::NodeHandle node_;
    ros::Subscriber tray_sub_;
};

Lv1Node::Lv1Node():
    lv1_(),
    tray_id_(1),
    node_(),
    tray_sub_()
{
    
}

void Lv1Node::Ini()
{
    if (lv1_.Open("/dev/ttyUSB0"))
    {
        tray_sub_ = node_.subscribe("motion", 100, &Lv1Node::Motion, this);
    }
    else
    {
        ROS_ERROR("open fail");
        exit(1);
    }
}

void Lv1Node::Motion(const std_msgs::String::ConstPtr &motion)
{
    if (motion->data == "sosogi0")
    {
        PlaySosogi(0);
    }
    else if (motion->data == "sosogi1")
    {
        PlaySosogi(1);
    }
    else if (motion->data == "sosogi2")
    {
        PlaySosogi(2);
    }
    else if (motion->data == "baibai")
    {
        static double tojiru[LV1_DOF] = {-80, 0, 0, 30, 0};
        static double hiraku[LV1_DOF] = {-30, 0, 0, -30, 0};
        SetJointAnglesWithoutTray(lv1_, hiraku, 1.0);
        SetJointAnglesWithoutTray(lv1_, tojiru, 1.0);
        SetJointAnglesWithoutTray(lv1_, hiraku, 1.0);
        SetJointAnglesWithoutTray(lv1_, tojiru, 1.0);
    }
    else if (motion->data == "dabadaba")
    {
        static double migi[LV1_DOF] = {-80, 0, 0, -30, 0};
        static double hidari[LV1_DOF] = {-30, 0, 0, 30, 0};
        SetJointAnglesWithoutTray(lv1_, migi, 1.0);
        SetJointAnglesWithoutTray(lv1_, hidari, 1.0);
        SetJointAnglesWithoutTray(lv1_, migi, 1.0);
        SetJointAnglesWithoutTray(lv1_, hidari, 1.0);
    }
    else if (motion->data == "reset")
    {
        static double reset[LV1_DOF] = {-50, 0, 0, 0, 0};
        SetJointAnglesWithoutTray(lv1_, reset, 1.0);
    }
    else{
        ROS_ERROR("no such motion %s", motion->data.c_str());
    }
}


void Lv1Node::PlaySosogi(int id)
{
    Sosogi(lv1_, id);
    sleep(1);
    SosogiBack(lv1_, id);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lv1_arm_node");

    Lv1Node l;
    l.Ini();

    ros::spin();

}

