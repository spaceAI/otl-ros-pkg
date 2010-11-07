#include <egeometry/coordinates.h>
#include <gtest/gtest.h>

using namespace egeometry;

void expect_near_vector(const FloatVector &v1, const FloatVector &v2)
{
    EXPECT_NEAR(v1[0], v2[0], 0.001);
    EXPECT_NEAR(v1[1], v2[1], 0.001);
    EXPECT_NEAR(v1[2], v2[2], 0.001);
}

void expect_near_matrix(const FloatMatrix &m1, const FloatMatrix &m2)
{
    expect_near_vector(m1[0], m2[0]);
    expect_near_vector(m1[1], m2[1]);
    expect_near_vector(m1[2], m2[2]);
}


TEST(NormalTest, Constructor)
{
    Coordinates c0;
    EXPECT_TRUE(c0.GetPosition() == FloatVector());
    EXPECT_TRUE(c0.GetRotation() == FloatMatrix::GetIdentity());
    EXPECT_TRUE(c0.GetName() == "");

    Coordinates c1("hoge");
    EXPECT_TRUE(c1.GetPosition() == FloatVector());
    EXPECT_TRUE(c1.GetRotation() == FloatMatrix::GetIdentity());
    EXPECT_TRUE(c1.GetName() == "hoge");

    Coordinates c2(FloatVector(0.1, 0.2, 0.3));

    EXPECT_TRUE(c2.GetPosition() == FloatVector(0.1, 0.2, 0.3));
    EXPECT_TRUE(c2.GetRotation() == FloatMatrix::GetIdentity());
    EXPECT_TRUE(c2.GetName() == "");

    double pos4[4] = {1.0, 0.0, -5.0, 0.0};
    Coordinates c4(pos4);
    EXPECT_TRUE(c4.GetPosition() == FloatVector(1.0, 0.0, -5.0));
    EXPECT_TRUE(c4.GetRotation() == FloatMatrix::GetIdentity());
    EXPECT_TRUE(c4.GetName() == "");

    FloatMatrix rot5(1, 2, 3,
                     4, 5, 6,
                     7, 8, 9);
    Coordinates c5(rot5);
    EXPECT_TRUE(c5.GetPosition() == FloatVector());
    EXPECT_TRUE(c5.GetRotation() == FloatMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9));
    EXPECT_TRUE(c5.GetName() == "");
    
    Coordinates c6(rot5, pos4, "c6");
    expect_near_vector(c6.GetPosition(), FloatVector(1.0, 0.0, -5.0));
    EXPECT_TRUE(c6.GetRotation() == FloatMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9));
    EXPECT_TRUE(c6.GetName() == "c6");

    Coordinates c7(c6);
    expect_near_vector(c7.GetPosition(), FloatVector(1.0, 0.0, -5.0));
    EXPECT_TRUE(c7.GetRotation() == FloatMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9));
    EXPECT_TRUE(c7.GetName() == "");
}

TEST(NormalTest, Operators)
{
    // operator=
    Coordinates c1(FloatVector(0.3, 0.5, 100.0));
    Coordinates c2(FloatVector(0.3, 0.5, 100.0));
    c1.SetName("hoge");
    c2.SetName("hage");
    EXPECT_TRUE(c1 == c2);
    EXPECT_FALSE(c1 != c2);
    c2.SetPosition(FloatVector(1, 2, 3));
    EXPECT_TRUE(c1 != c2);
    EXPECT_FALSE(c1 == c2);

    Coordinates c3 = c1;
    EXPECT_TRUE(c3 == c1);

    Coordinates c4;
    c4 = c1;
    EXPECT_TRUE(c4 == c1);

    EXPECT_FALSE(c4.GetName() == "ahoge");
    c4.SetName("ahoge");
    EXPECT_TRUE(c4.GetName() == "ahoge");

    EXPECT_FALSE(c4.GetRotation() == FloatMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9));
    c4.SetRotation(FloatMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9));
    EXPECT_TRUE(c4.GetRotation() == FloatMatrix(1, 2, 3, 4, 5, 6, 7, 8, 9));

    c4.SetRotation(-1, 0, 1, 0, -1, 0, 1, 1, 1);
    EXPECT_TRUE(c4.GetRotation() == FloatMatrix(-1, 0, 1, 0, -1, 0, 1, 1, 1));

    c4.SetPosition(FloatVector(100, 200, -300));
    EXPECT_TRUE(c4.GetPosition() == FloatVector(100, 200, -300));

    c4.SetPosition(-500, 600, -1000);
    EXPECT_TRUE(c4.GetPosition() == FloatVector(-500, 600, -1000));

    Coordinates c5("c5");
    c5.SetCoords(c4);
    EXPECT_TRUE(c4 == c5);
    
    Coordinates c6("c6");
    c6.SetCoords(c5.GetRotation(), c5.GetPosition());
    EXPECT_TRUE(c6 == c5);
    
}

TEST(NormalTest, World)
{
    Coordinates c1;
    c1.SetPosition(100, 200, -100);
    c1.SetRotation(1, 0, 0, 0, -1, 0, 0, 0, 1);
    EXPECT_TRUE(c1.GetWorldRotation() == c1.GetRotation());
    EXPECT_TRUE(c1.GetWorldPosition() == c1.GetPosition());

    FloatVector pos1;
    FloatMatrix rot1;
    c1.GetWorldPosition(pos1);
    c1.GetWorldRotation(rot1);
    EXPECT_TRUE(pos1 == c1.GetPosition());
    EXPECT_TRUE(rot1 == c1.GetRotation());
    EXPECT_TRUE(pos1 == c1.GetWorldPosition());
    // GetWorldCoords
    EXPECT_TRUE(c1.GetWorldCoords() == c1);
    Coordinates wc;
    c1.GetWorldCoords(wc);
    EXPECT_TRUE(wc == c1);

    wc.ResetCoords();
    EXPECT_TRUE(wc.GetPosition() == FloatVector());
    EXPECT_TRUE(wc.GetRotation() == FloatMatrix::GetIdentity());
    
    EXPECT_TRUE(c1.GetParent() == Coordinates::world_coords);
    
}

TEST(NormalTest, Move)
{
    // rotate local
    Coordinates c1(FloatVector(100, 200, 100));
    c1.Rotate(DegToRad(30), FloatVector(0, 1, 0));
    EXPECT_TRUE(c1.GetPosition() == FloatVector(100, 200, 100));

    expect_near_matrix(c1.GetRotation(), FloatMatrix(0.866026, 0.0, 0.5,
                                                     0.0, 1.0, 0.0,
                                                     -0.5, 0.0, 0.866026));

    // rotate world
    Coordinates c1w(FloatVector(100, 200, 100));
    c1w.Rotate(DegToRad(-30), FloatVector(-1, 0, 1));
    c1w.Rotate(DegToRad(-30), FloatVector(0, 1, 1), kWorld);
    expect_near_matrix(c1w.GetRotation(), FloatMatrix(0.706697, 0.487372, -0.512882,
                                                      -0.664227, 0.706697, -0.243686,
                                                      0.243686, 0.512882, 0.823146));
    
    // orient local
    Coordinates co1(FloatVector(100, 200, 100));
    co1.Orient(DegToRad(30), FloatVector(0, 1, 0));
    EXPECT_TRUE(co1.GetPosition() == FloatVector(100, 200, 100));
    expect_near_matrix(co1.GetRotation(), FloatMatrix(0.866026, 0.0, 0.5,
                                                      0.0, 1.0, 0.0,
                                                      -0.5, 0.0, 0.866026));


    // orient world
    co1.Orient(DegToRad(30), FloatVector(0, 1, 0), kWorld);
    EXPECT_TRUE(co1.GetPosition() == FloatVector(100, 200, 100));
    expect_near_matrix(co1.GetRotation(), FloatMatrix(0.866026, 0.0, 0.5,
                                                      0.0, 1.0, 0.0,
                                                      -0.5, 0.0, 0.866026));

    // translate local
    Coordinates ct1(FloatVector(100, 200, 100));
    ct1.Rotate(DegToRad(40), FloatVector(0, 1, 0));
    ct1.Translate(FloatVector(100, 50, -20));
    expect_near_vector(ct1.GetPosition(), FloatVector(163.749, 250, 20.4004));
    ct1.Translate(FloatVector(100, 50, -20));
    expect_near_vector(ct1.GetPosition(), FloatVector(227.497, 300, -59.1992));

    // translate world
    Coordinates ct2(FloatVector(100, 200, 100));
    ct2.Rotate(DegToRad(40), FloatVector(0, 1, 0));
    ct2.Translate(FloatVector(100, 50, -20), kWorld);
    expect_near_vector(ct2.GetPosition(), FloatVector(200, 250, 80));
    ct2.Translate(FloatVector(100, 50, -20), kWorld);
    expect_near_vector(ct2.GetPosition(), FloatVector(300, 300, 60));
    
    // locate local
    Coordinates cl1(FloatVector(100, 200, 100));
    cl1.Rotate(DegToRad(40), FloatVector(0, 1, 0));
    cl1.Locate(FloatVector(100, 50, -20));
    expect_near_vector(cl1.GetPosition(), FloatVector(163.749, 250, 20.4004));
    cl1.Locate(FloatVector(100, 50, -20));
    expect_near_vector(cl1.GetPosition(), FloatVector(227.497, 300, -59.1992));
    
    // locate world
    Coordinates cl2(FloatVector(100, 200, 100));
    cl2.Rotate(DegToRad(40), FloatVector(0, 1, 0));
    cl2.Locate(FloatVector(100, 50, -20), kWorld);
    expect_near_vector(cl2.GetPosition(), FloatVector(100, 50, -20));
    cl2.Locate(FloatVector(100, 50, -20), kWorld);
    expect_near_vector(cl2.GetPosition(), FloatVector(100, 50, -20));

    // transform vector
    FloatVector v1;
    c1.GetTransformVector(FloatVector(100, 60, -80), v1);
    expect_near_vector(v1, FloatVector(146.603, 260.0, -19.282));

    c1.Rotate(DegToRad(-30), FloatVector(0, 1, -1));
    c1.GetTransformVector(FloatVector(-80, -10, 30), v1);
    expect_near_vector(v1, FloatVector(34.0643, 160.376, 138.502));

    // inverse transform vector
    Coordinates c2(FloatVector(-50.0, 10.0, 0.0));
    c2.Rotate(DegToRad(30), FloatVector(-1, 0, 1));
    FloatVector v2;
    c2.GetInverseTransformVector(FloatVector(100, -500, 30), v2);
    expect_near_vector(FloatVector(-42.3698, -505.313, -162.37), v2);
    
    // GetParentVector
    Coordinates cp1(FloatVector(100, 200, 100));
    cp1.Rotate(DegToRad(270), FloatVector(0, 1, -1));
    FloatVector vr1;
    // local
    cp1.GetParentVector(FloatVector(100, -50, 10), vr1, kLocal);
    expect_near_vector(FloatVector(128.284, 240.711, 200.711), vr1);
    // world
    cp1.GetParentVector(FloatVector(100, -50, 10), vr1, kWorld);
    expect_near_vector(FloatVector(100, -50, 10), vr1);
    // orientation local
    cp1.GetParentOrientation(FloatVector(100, -50, 10), vr1, kLocal);
    expect_near_vector(FloatVector(28.2841, 40.711, 100.711), vr1);
    // orientation world
    cp1.GetParentOrientation(FloatVector(100, -50, 10), vr1, kWorld);
    expect_near_vector(FloatVector(100, -50, 10), vr1);
    
    // set rpy
    Coordinates rpy;
    rpy.SetRPY(0.1, 1.5, 100);
    expect_near_matrix(FloatMatrix(0.070384, -0.588662, 0.805309,
                                   0.007062, 0.807585, 0.589709,
                                   -0.997495, -0.035819, 0.060998) ,
                       rpy.GetRotation());

    // get rpy
    Coordinates rpy2;
    rpy2.Rotate(DegToRad(30), FloatVector(1, 0, 0));
    rpy2.Rotate(DegToRad(-90), FloatVector(0, 1, 0));
    rpy2.Rotate(DegToRad(1), FloatVector(0, 1, 1));
    double r, p, y;
    rpy2.GetRPY(r, p, y);
    EXPECT_NEAR(r, -1.5458, 0.001);
    EXPECT_NEAR(p, -1.0594, 0.001);
    EXPECT_NEAR(y, 1.5488, 0.001);
    double r2, p2, y2;
    rpy2.GetRPY(r2, p2, y2, false);
    EXPECT_NEAR(r2, 1.59602, 0.001);
    EXPECT_NEAR(p2, -2.08219, 0.001);
    EXPECT_NEAR(y2, -1.59271, 0.001);

    // get quaternion
    double q1[4];
    rpy2.GetQuaternion(q1);
    //
    rpy2.SetQuaternion(q1);
    rpy2.GetRPY(r, p, y);
    EXPECT_NEAR(r, -1.5458, 0.001);
    EXPECT_NEAR(p, -1.0594, 0.001);
    EXPECT_NEAR(y, 1.5488, 0.001);
    
}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
