#include <egeometry/e2msg.h>
#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <iostream>

using namespace egeometry;
using namespace geometry_msgs;

TEST(NormalTest, TF)
{
    Coordinates c1("c1");
    c1.Locate(FloatVector(1.0, 2.0, 3.0));
    c1.Rotate(DegToRad(30.0), kAxisX);
    c1.Rotate(DegToRad(20.0), kAxisY);

    TransformStamped tf1;
    double q[4];

    CoordinatesToTFMsg(c1, tf1);

    EXPECT_FLOAT_EQ(tf1.transform.translation.x, c1.GetPosition().GetX());
    EXPECT_FLOAT_EQ(tf1.transform.translation.y, c1.GetPosition().GetY());
    EXPECT_FLOAT_EQ(tf1.transform.translation.z, c1.GetPosition().GetZ());
    c1.GetQuaternion(q);
    EXPECT_NEAR(q[0], tf1.transform.rotation.x, 0.0001);
    EXPECT_NEAR(q[1], tf1.transform.rotation.y, 0.0001);
    EXPECT_NEAR(q[2], tf1.transform.rotation.z, 0.0001);
    EXPECT_NEAR(q[3], tf1.transform.rotation.w, 0.0001);
    
    TFMsgToCoordinates(tf1, c1);

    EXPECT_FLOAT_EQ(tf1.transform.translation.x, c1.GetPosition().GetX());
    EXPECT_FLOAT_EQ(tf1.transform.translation.y, c1.GetPosition().GetY());
    EXPECT_FLOAT_EQ(tf1.transform.translation.z, c1.GetPosition().GetZ());
    c1.GetQuaternion(q);
    EXPECT_NEAR(q[0], tf1.transform.rotation.x, 0.0001);
    EXPECT_NEAR(q[1], tf1.transform.rotation.y, 0.0001);
    EXPECT_NEAR(q[2], tf1.transform.rotation.z, 0.0001);
    EXPECT_NEAR(q[3], tf1.transform.rotation.w, 0.0001);
}

TEST(NormalTest, Pose)
{
    Coordinates c1("c1");
    c1.Locate(FloatVector(1.0, -10.0, 3.0));
    c1.Rotate(DegToRad(30.0), kAxisX);
    c1.Rotate(DegToRad(-20.0), kAxisY);

    PoseStamped p1;
    double q[4];

    CoordinatesToPoseMsg(c1, p1);

    EXPECT_FLOAT_EQ(p1.pose.position.x, c1.GetPosition().GetX());
    EXPECT_FLOAT_EQ(p1.pose.position.y, c1.GetPosition().GetY());
    EXPECT_FLOAT_EQ(p1.pose.position.z, c1.GetPosition().GetZ());
    c1.GetQuaternion(q);
    EXPECT_NEAR(q[0], p1.pose.orientation.x, 0.0001);
    EXPECT_NEAR(q[1], p1.pose.orientation.y, 0.0001);
    EXPECT_NEAR(q[2], p1.pose.orientation.z, 0.0001);
    EXPECT_NEAR(q[3], p1.pose.orientation.w, 0.0001);
    
    PoseMsgToCoordinates(p1, c1);

    EXPECT_FLOAT_EQ(p1.pose.position.x, c1.GetPosition().GetX());
    EXPECT_FLOAT_EQ(p1.pose.position.y, c1.GetPosition().GetY());
    EXPECT_FLOAT_EQ(p1.pose.position.z, c1.GetPosition().GetZ());
    c1.GetQuaternion(q);
    EXPECT_NEAR(q[0], p1.pose.orientation.x, 0.0001);
    EXPECT_NEAR(q[1], p1.pose.orientation.y, 0.0001);
    EXPECT_NEAR(q[2], p1.pose.orientation.z, 0.0001);
    EXPECT_NEAR(q[3], p1.pose.orientation.w, 0.0001);
}

TEST(NormalTest, Rotation)
{
    FloatMatrix m1;
    Quaternion q1;
    SetRPYAngles(m1, 1.0, 2.0, 3.0);
    RotationToQuaternionMsg(m1, q1);
//    std::cerr << q1.x << " " << q1.y<< " " << q1.z<< " " << q1.w << std::endl;

    EXPECT_NEAR(-0.444435, q1.x, 0.0001);
    EXPECT_NEAR(-0.310622, q1.y, 0.0001);
    EXPECT_NEAR(0.718287, q1.z, 0.0001);
    EXPECT_NEAR(0.435953, q1.w, 0.0001);
    
    QuaternionMsgToRotation(q1, m1);

    double r, p, y;
    GetRPYAngles(m1, r, p, y, false);
    EXPECT_NEAR(1.0, r, 0.0001);
    EXPECT_NEAR(2.0, p, 0.0001);
    EXPECT_NEAR(3.0, y, 0.0001);
}

TEST(NormalTest, Position)
{
    FloatVector f1(-1.0, 2.0, 0.1);
    Point p1;
    PositionToPointMsg(f1, p1);
    EXPECT_FLOAT_EQ(f1[0], p1.x);
    EXPECT_FLOAT_EQ(f1[1], p1.y);;
    EXPECT_FLOAT_EQ(f1[2], p1.z);

    PointMsgToPosition(p1, f1);
    EXPECT_FLOAT_EQ(p1.x, f1[0]);
    EXPECT_FLOAT_EQ(p1.y, f1[1]);
    EXPECT_FLOAT_EQ(p1.z, f1[2]);
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
