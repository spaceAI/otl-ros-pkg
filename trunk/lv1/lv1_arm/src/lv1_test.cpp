#include <lv1_arm/lv1.h>
#include <stdlib.h>
#include <iostream>

int main(int argc, char *argv[])
{
    if (argc < LV1_DOF +1)
    {
        std::cerr << "usage: lv1_test j1 j2 j3 j4 j5 tm" << std::endl;
        return 1;
    }
    double pos[LV1_DOF];
    for (int i=1; i<=LV1_DOF; i++)
    {
        pos[i-1] = atof(argv[i]);
    }
    
    Lv1Interface lv;
    lv.Open("/dev/ttyUSB0");
    sleep(2);
    lv.setJointAngles(pos, atof(argv[LV1_DOF+1]));
}

