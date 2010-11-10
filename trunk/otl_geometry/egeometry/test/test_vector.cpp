#include <egeometry/vector3.h>
#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <iostream>

using namespace egeometry;

TEST(NormalTest, MemberFunction)
{
    FloatVector v1(3, 1, 2);
    FloatVector v2(-5, -1, 1);
    // operator[]
    EXPECT_FLOAT_EQ(3, v1[0]);
    EXPECT_FLOAT_EQ(1, v1[1]);
    EXPECT_FLOAT_EQ(2, v1[2]);

    // set value by operator[]
    FloatVector v0;
    v0[0] = 1.1;
    EXPECT_DOUBLE_EQ(1.1, v0[0]);

    // oeprator +=
    v1 += v2;
    EXPECT_FLOAT_EQ(-2, v1[0]);
    EXPECT_FLOAT_EQ(0, v1[1]);
    EXPECT_FLOAT_EQ(3, v1[2]);

    FloatVector v3(3, 1, 2);
    FloatVector v4(-5, -1, 1);
    // operator -=
    v4 -= v3;
    EXPECT_FLOAT_EQ(-8, v4[0]);
    EXPECT_FLOAT_EQ(-2, v4[1]);
    EXPECT_FLOAT_EQ(-1, v4[2]);

    // operator -=
    v3 *= 0.1;
    EXPECT_FLOAT_EQ(0.3, v3[0]);
    EXPECT_FLOAT_EQ(0.1, v3[1]);
    EXPECT_FLOAT_EQ(0.2, v3[2]);
    
    // operator /=
    FloatVector v5;
    // SetValue
    v5.SetValue(0.5, -0.1, 1.5);
    v5 /= 2.0;
    EXPECT_FLOAT_EQ(0.25, v5[0]);
    EXPECT_FLOAT_EQ(-0.05,v5[1]);
    EXPECT_FLOAT_EQ(0.75, v5[2]);

    // Dot
    FloatVector v6(1, 2, 3);
    FloatVector v7(-1, -0.5, 0.6);
    EXPECT_FLOAT_EQ(-0.2, v6.Dot(v7));
    EXPECT_FLOAT_EQ(-0.2, v6.Dot(v7));
    
    // Length2
    EXPECT_FLOAT_EQ(1+4+9, v6.Length2());

    // Length
    FloatVector v8(1, -2, -3);
    EXPECT_FLOAT_EQ(sqrt(1+4+9), v8.Length());
    
    // Distance2
    FloatVector v9(1, -2, -3);
    FloatVector v10(2, 2, 0);
    double dist2 = v9.Distance2(v10);
    EXPECT_FLOAT_EQ(1+16+9, dist2);

    // Distance
    double dist = v9.Distance(v10);
    EXPECT_FLOAT_EQ(sqrt(1+16+9), dist);

    // Normalize
    // GetX,Y,Z
    double len = v9.Length();
    v9.Normalize();
    EXPECT_FLOAT_EQ(1/len, v9.GetX());
    EXPECT_FLOAT_EQ(-2/len, v9.GetY());
    EXPECT_FLOAT_EQ(-3/len, v9.GetZ());

    // SetX, Y, Y
    v9.SetX(-100);
    EXPECT_FLOAT_EQ(-100, v9.GetX());
    v9.SetY(10);
    EXPECT_FLOAT_EQ(10, v9.GetY());
    v9.SetZ(0);
    EXPECT_FLOAT_EQ(0, v9.GetZ());

    // operator==
    FloatVector va(1, 2, 3);
    FloatVector vb(1, 2, 3);
    EXPECT_TRUE(va == vb);

    // operator!=
    vb.SetX(0.9);
    EXPECT_TRUE(va != vb);

    // const
    const FloatVector vc(10, -1, 0.2);
    EXPECT_EQ(10, vc[0]);
    EXPECT_EQ(-1, vc.GetY());
    
    // set value
    FloatVector vd(10, -1, 0.2);
    vd.SetValue(0.1, 0.2, -0.1);
    EXPECT_EQ(0.1, vd[0]);
    EXPECT_EQ(0.2, vd[1]);
    EXPECT_EQ(-0.1, vd[2]);

    // set zeo
    vd.SetZero();
    EXPECT_EQ(0.0, vd[0]);
    EXPECT_EQ(0.0, vd[1]);
    EXPECT_EQ(0.0, vd[2]);
}

TEST(NormalTest, NonMemberFunction)
{
    // operator-
    FloatVector v1;
    v1.SetValue(100, 2, 3);
    FloatVector v2;
    v2.SetValue(50, -20, 100);
    FloatVector v3(50, 22, -97);
    EXPECT_TRUE((v1 - v2) == v3);

    // operator-
    FloatVector v4(-50, -22, 97);
    EXPECT_TRUE(-v4 == v3);

    // operator+
    FloatVector v5(0.1, -0.1, 0.0);
    FloatVector v6(0.8, 0.9, 1.0);
    FloatVector v7(0.9, 0.8, 1.0);
    EXPECT_TRUE((v5+v6)==v7);
    
    // operator*
    FloatVector v8(0.08, -0.09, 0.0);

    FloatVector v56 = v5 * v6;
    EXPECT_FLOAT_EQ(0.08, v56[0]);
    EXPECT_FLOAT_EQ(-0.09, v56[1]);
    EXPECT_FLOAT_EQ(0.0, v56[2]);
    //EXPECT_TRUE(v56==v8);

    FloatVector v9(1.0, -1.0, 0.0);
    EXPECT_TRUE((v5 * 10.0)==v9);

    EXPECT_TRUE((10.0 * v5)==v9);
    FloatVector v10(0.4, 0.45, 0.5);
    // oeprator/
    FloatVector v11 = v6/2.0;
    EXPECT_TRUE(v11 == v10);
    
}

#include<vector>

TEST(NormalTest, Template)
{
    std::vector<double> v0;
    v0.push_back(0.1);
    v0.push_back(0.5);
    v0.push_back(-0.1);

    FloatVector v3_r(0.1, 0.5, -0.1);
    FloatVector v3_0 = v0;
    EXPECT_TRUE(v3_0==v3_r);

    FloatVector v3_1;
    v3_1 = v0;
    EXPECT_TRUE(v3_1==v3_r);
    
    double vd[4];
    vd[0] = 0.1;
    vd[1] = 0.5;
    vd[2] = -0.1;

    FloatVector v3_2;
    v3_2 = vd;
    EXPECT_TRUE(v3_2==v3_r);
    FloatVector v3_3 = vd;
    EXPECT_TRUE(v3_2==v3_r);

    int vi[4];
    vi[0] = 1;
    vi[1] = 5;
    vi[2] = -2;

    FloatVector v3_r2(1, 5, -2);
    FloatVector v3_4 = vi;
    EXPECT_FLOAT_EQ(v3_r2[0], v3_4[0]);
    EXPECT_FLOAT_EQ(v3_r2[1], v3_4[1]);
    EXPECT_FLOAT_EQ(v3_r2[2], v3_4[2]);


    double vf[4];
    vf[0] = 0.1;
    vf[1] = 0.5;
    vf[2] = -0.1;

    FloatVector v3_5;
    v3_5 = vf;
    EXPECT_TRUE(v3_5==v3_r);
    FloatVector v3_6 = vf;
    EXPECT_TRUE(v3_6==v3_r);

}


int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

