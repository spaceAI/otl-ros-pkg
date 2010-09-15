#include <lv1_arm/lv1.h>
#include <iostream>

int main(int argc, char *argv[])
{
    Lv1Interface lv;
    lv.Open("/dev/ttyUSB0");
    if (argc <2)
    {
        std::cerr << "usage: lv1_main id" <<std::endl;
        exit(1);
    }

    int i = atoi(argv[1]);
    Sosogi(lv, i);
    sleep(1);
    SosogiBack(lv, i);
}

