#include <egeometry/matrix33.h>
#include <gtest/gtest.h>
#include <gtest/gtest-spi.h>
#include <iostream>

using namespace egeometry;

TEST(NormalTest, MemberFunction)
{
    Matrix33 aa;
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            EXPECT_FLOAT_EQ(0, aa[i][j]);
        }
    }
    
    Matrix33 bb(0,1,2,
                3,4,5,
                6,7,8);
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            double d = static_cast<double>(i*3+j);
            EXPECT_FLOAT_EQ(d, bb[i][j]);
        }
    }

    Matrix33 cc(bb);
    Matrix33 dd;
    // operator=
    dd = cc;
    // operator==
    EXPECT_TRUE(bb == cc);
    EXPECT_TRUE(dd == cc);
    // GetColumn
    Vector3 v0(dd.GetColumn(0));
    EXPECT_FLOAT_EQ(0, v0[0]);
    EXPECT_FLOAT_EQ(3, v0[1]);
    EXPECT_FLOAT_EQ(6, v0[2]);

    Vector3 v1(dd.GetColumn(1));
    EXPECT_FLOAT_EQ(1, v1[0]);
    EXPECT_FLOAT_EQ(4, v1[1]);
    EXPECT_FLOAT_EQ(7, v1[2]);

    Vector3 v2(dd.GetColumn(2));
    EXPECT_FLOAT_EQ(2, v2[0]);
    EXPECT_FLOAT_EQ(5, v2[1]);
    EXPECT_FLOAT_EQ(8, v2[2]);

    //not change
    v2.SetX(-5.0);
    EXPECT_FALSE(v2 == dd.GetColumn(2));

    // GetRow
    v0 = dd.GetRow(0);
    EXPECT_FLOAT_EQ(0, v0[0]);
    EXPECT_FLOAT_EQ(1, v0[1]);
    EXPECT_FLOAT_EQ(2, v0[2]);

    v1 = dd.GetRow(1);
    EXPECT_FLOAT_EQ(3, v1[0]);
    EXPECT_FLOAT_EQ(4, v1[1]);
    EXPECT_FLOAT_EQ(5, v1[2]);

    v2 = dd.GetRow(2);
    EXPECT_FLOAT_EQ(6, v2[0]);
    EXPECT_FLOAT_EQ(7, v2[1]);
    EXPECT_FLOAT_EQ(8, v2[2]);
    
    //not change
    v2.SetY(-5.0);
    EXPECT_FALSE(v2 == dd.GetRow(2));

    // operator[]
    Vector3 &r0 = dd[0];
    EXPECT_FLOAT_EQ(0.0 , r0[0]);
    EXPECT_FLOAT_EQ(1.0 , r0[1]);
    EXPECT_FLOAT_EQ(2.0 , r0[2]);

    Vector3 &r1 = dd[1];
    EXPECT_FLOAT_EQ(3.0 , r1[0]);
    EXPECT_FLOAT_EQ(4.0 , r1[1]);
    EXPECT_FLOAT_EQ(5.0 , r1[2]);

    Vector3 &r2 = dd[2];
    EXPECT_FLOAT_EQ(6.0 , r2[0]);
    EXPECT_FLOAT_EQ(7.0 , r2[1]);
    EXPECT_FLOAT_EQ(8.0 , r2[2]);

    const Matrix33 cm = dd;

    const Vector3 &cr0 = cm[0];
    EXPECT_FLOAT_EQ(0.0 , cr0[0]);
    EXPECT_FLOAT_EQ(1.0 , cr0[1]);
    EXPECT_FLOAT_EQ(2.0 , cr0[2]);

    const Vector3 &cr1 = cm[1];
    EXPECT_FLOAT_EQ(3.0 , cr1[0]);
    EXPECT_FLOAT_EQ(4.0 , cr1[1]);
    EXPECT_FLOAT_EQ(5.0 , cr1[2]);

    const Vector3 &cr2 = cm[2];
    EXPECT_FLOAT_EQ(6.0 , cr2[0]);
    EXPECT_FLOAT_EQ(7.0 , cr2[1]);
    EXPECT_FLOAT_EQ(8.0 , cr2[2]);

    // operator*= & tdotx,y,z
    Matrix33 m1(0.984808, -0.17101, -0.030154,
              0.173648, 0.969846, 0.17101,
              0.0, -0.173648, 0.984808);
    Matrix33 m2(0.866026, 0.0, 0.5,
                0.25, 0.866026, -0.433013,
                -0.433013, 0.5, 0.75);
    m1 *= m2;
    EXPECT_NEAR(0.823174, m1[0][0], 0.0001);
    EXPECT_NEAR(-0.163176, m1[0][1], 0.0001);
    EXPECT_NEAR(0.543838, m1[0][2], 0.0001);
    EXPECT_NEAR(0.318796, m1[1][0], 0.0001);
    EXPECT_NEAR(0.925417, m1[1][1], 0.0001);
    EXPECT_NEAR(-0.204874, m1[1][2], 0.0001);
    EXPECT_NEAR(-0.469847, m1[2][0], 0.0001);
    EXPECT_NEAR(0.34202, m1[2][1], 0.0001);
    EXPECT_NEAR(0.813798, m1[2][2], 0.0001);

    // inverse
    Matrix33 mi = m1.Inverse();
    EXPECT_NEAR(0.823174, mi[0][0], 0.0001);
    EXPECT_NEAR(-0.163176, mi[1][0], 0.0001);
    EXPECT_NEAR(0.543838, mi[2][0], 0.0001);
    EXPECT_NEAR(0.318796, mi[0][1], 0.0001);
    EXPECT_NEAR(0.925417, mi[1][1], 0.0001);
    EXPECT_NEAR(-0.204874, mi[2][1], 0.0001);
    EXPECT_NEAR(-0.469847, mi[0][2], 0.0001);
    EXPECT_NEAR(0.34202, mi[1][2], 0.0001);
    EXPECT_NEAR(0.813798, mi[2][2], 0.0001);
    
    // Transpose
    Matrix33 mt = m1.Transpose();
    EXPECT_NEAR(0.823174, mt[0][0], 0.0001);
    EXPECT_NEAR(-0.163176, mt[1][0], 0.0001);
    EXPECT_NEAR(0.543838, mt[2][0], 0.0001);
    EXPECT_NEAR(0.318796, mt[0][1], 0.0001);
    EXPECT_NEAR(0.925417, mt[1][1], 0.0001);
    EXPECT_NEAR(-0.204874, mt[2][1], 0.0001);
    EXPECT_NEAR(-0.469847, mt[0][2], 0.0001);
    EXPECT_NEAR(0.34202, mt[1][2], 0.0001);
    EXPECT_NEAR(0.813798, mt[2][2], 0.0001);

    // identity
    Matrix33 im = Matrix33::GetIdentity();
    for (int i = 0; i<3; i++)
    {
        for (int j = 0; j<3; j++)
        {
            if ( i == j)
            {
                EXPECT_DOUBLE_EQ(1.0, im[i][j]);
            }
            else
            {
                EXPECT_DOUBLE_EQ(0.0, im[i][j]);
            }
        }
    }
    // set zero
    im.SetZero();
    for (int i = 0; i<3; i++)
    {
        for (int j = 0; j<3; j++)
        {
            EXPECT_DOUBLE_EQ(0.0, im[i][j]);
        }
    }
}

TEST(NormalTest, NonMemberFunction)
{
    Matrix33 m1(0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0);
    Matrix33 m2(-0.1, 0.2, 0.3, -0.4, 0.5, -0.6, 0.1, 0.2, 0.1);
    Vector3 v1(100, 0.5, -5.0);
    // operator*
    EXPECT_TRUE(m1 * v1 == Vector3(-9.5, 277.0, 563.5));
    EXPECT_TRUE(v1 * m1 == Vector3(-28.5, 67.0, 162.5));
    Matrix33 m1m2(m1 * m2);
    EXPECT_NEAR(-0.2, m1m2[0][0], 0.000001);
    EXPECT_NEAR( 0.9, m1m2[0][1], 0.000001);
    EXPECT_NEAR(-0.4, m1m2[0][2], 0.000001);
    EXPECT_NEAR(-1.4, m1m2[1][0], 0.000001);
    EXPECT_NEAR( 3.6, m1m2[1][1], 0.000001);
    EXPECT_NEAR(-1.0, m1m2[1][2], 0.000001);
    EXPECT_NEAR(-2.6, m1m2[2][0], 0.000001);
    EXPECT_NEAR(6.3, m1m2[2][1], 0.000001);
    EXPECT_NEAR(-1.6, m1m2[2][2], 0.000001);

    Matrix33 m2m1(m2 * m1);
    EXPECT_NEAR(2.4, m2m1[0][0], 0.000001);
    EXPECT_NEAR(2.8, m2m1[0][1], 0.000001);
    EXPECT_NEAR(3.2, m2m1[0][2], 0.000001);
    EXPECT_NEAR(-2.1, m2m1[1][0], 0.000001);
    EXPECT_NEAR(-2.6, m2m1[1][1], 0.000001);
    EXPECT_NEAR(-3.1, m2m1[1][2], 0.000001);
    EXPECT_NEAR(1.2, m2m1[2][0], 0.000001);
    EXPECT_NEAR(1.6, m2m1[2][1], 0.000001);
    EXPECT_NEAR(2.0, m2m1[2][2], 0.000001);

}

int main(int argc, char *argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

