#include <lv1_arm/lv1.h>
#include <iostream>
#include <sstream>
#include <string>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// シリアルデバイスをオープンして初期化
Lv1Interface::Lv1Interface()
{
}

bool Lv1Interface::Open(const std::string &devName)
{
    double ang[LV1_DOF];

    // open serial (read & write)
    fd = open(devName.c_str(), O_RDWR | O_NOCTTY);

    // fail to open
    if (fd < 0)
    {
        std::cout << "failed to open " << devName << std::endl;
        return false;
    }

    setSerialPort();
    std::cout << "opened " << devName << std::endl;

    setInitialAngles();
    //getJointAngles(ang);
//    setJointAnglesInternal(ang);

    return true;
}

void Lv1Interface::setSerialPort()
{
    struct termios newtio;
    tcgetattr(fd, &oldtio);
    memset(&newtio,0,sizeof(newtio));
    
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = B9600 | CS8 | CLOCAL | CREAD ;
    newtio.c_iflag = 0;
    newtio.c_cc[VTIME] = 0;
    //    newtio.c_cc[VMIN] = 1;
    newtio.c_cc[VMIN] = 0;
    // set speed
    cfsetospeed(&newtio, B9600);
    cfsetispeed(&newtio, B9600);
    
    tcflush(fd, TCIFLUSH);
    
    // 新しい設定を適用
    //   tcsetattr(fd, TCSANOW,&newtio);
    if ( tcsetattr(fd, TCSAFLUSH, &newtio) != 0 )
    {
        std::cout << "fail to set serial port" << std::endl;
    }
}
  
void Lv1Interface::setInitialAngles()
{
    
    offset[0] = -10;
    offset[1] = 15;
//    offset[2] = -20;
    offset[2] = 25;
    offset[3] = 0;
    offset[4] = -10;
    
    angles[0] = -50;
    
    angles[1] = 0;
    angles[2] = 0;
    angles[3] = 0;
    
    //angles[4] = -45;
    angles[4] = 0;
    
    // servo on
    for (int i = 0; i < LV1_DOF; i++)
    {
        servo[i] = 1;
    }
    // -50 0 0 kamae
    // 100 -75 100 open
    // 100 0 100 pre-hold
    // 100 0 0 hold
    // -64 0 0 forward
    // -64 -90 0 sosogi
    // -64 0 0 modosu
    // set limit angles (High)
    
    loAngleLimits[0] =  -80;
    hiAngleLimits[0] =  100;
    
    loAngleLimits[1] = -100;
    hiAngleLimits[1] =  100;
    
    loAngleLimits[2] = -92;
    hiAngleLimits[2] =  92;
    
    loAngleLimits[3] = -90;
    hiAngleLimits[3] =  90;
    
    loAngleLimits[4] = -180;
    hiAngleLimits[4] =  180;
    
    //loAngleLimits[5] = -92;
    //hiAngleLimits[5] =  92;
    
}

bool Lv1Interface::angleLimitFilter(double *ang, int i)
{
    bool ret = false;
    if ( *ang > hiAngleLimits[i] )
    {
        *ang = hiAngleLimits[i];
        ret = true;
    }
    else if ( *ang < loAngleLimits[i] )
    {
        *ang = loAngleLimits[i];
        ret = true;
    }
    return ret;
}

bool Lv1Interface::checkAngleLimits(double *ang)
{
    bool ret = false;
    
    for (int i = 0; i < LV1_DOF; i++)
    {
        if (angleLimitFilter(&ang[i], i))
	{
            ret = true;
	}
    }
    return ret;
}

bool Lv1Interface::setJointAngles(double *ang, double time)
{
    if (checkAngleLimits(ang))
    {
        std::cerr << "limit over" << std::endl;
    }
    // 20msec で補完
    double interpolate_time = 50.0;
    int n = (int)time * (1000 / interpolate_time);
    std::cout << "times = " << n  << std::endl;
    double tangles[LV1_DOF];
    double v[LV1_DOF];
    bool ret = true;

    for (int i = 0; i<LV1_DOF; i++)
    {
        v[i] = (ang[i] - angles[i]) / n;
    }

    for (int j = 1; j <= n; j++)
    {
        for (int i = 0; i<LV1_DOF; i++)
        {
            tangles[i] = angles[i] + (v[i] * j);
        }
        ret = setJointAnglesInternal(tangles);
        usleep(1000 * (interpolate_time - 2));
    }

    for (int i = 0; i<LV1_DOF; i++)
    {
        angles[i] = ang[i];
    }

    return ret;
}


bool Lv1Interface::setJointAnglesInternal(double *ang)
{
    int wroteByte = 0;
    int tangles[LV1_DOF]; // 書き込み用バッファ
    
    for (int i = 0; i < LV1_DOF; i++)
    {
        // 角度の保存
        //angles[i] = (double)ang[i];
	
        // tanglesは 1-255
            tangles[i] = (int)(((int)ang[i] + offset[i] + 135) * 180 / 270);
        if (tangles[i] > 180)
        {
            tangles[i] = 180;
        }else if (tangles[i] <= 0)
        {
            tangles[i] = 0;
        }
    }
    std::stringstream ss;
    ss << "a";

    for (int i = 0; i < LV1_DOF-1; i++)
    {
        ss << tangles[i] << ",";
    }

    ss << tangles[LV1_DOF-1] << "E";
  
    const char *cbuf = ss.str().c_str();
    char rbuf[255];
    int len = ss.str().size();
    int ret = 0;
    
    if ( fd > 0)
    {
        while(wroteByte < len)
	{
            ret = write(fd, &(cbuf[wroteByte]), len- wroteByte);
            if (ret > 0)
	    {
                wroteByte += ret;
	    }
            else
	    {
                std::cout << "failed to write" << std::endl;
                return false;
	    }
	}
        ssize_t a;
        a = read(fd, &(rbuf[0]), 255);
        if ( rbuf[0] == 'C' )
        {
            std::cout << "success to read" << std::endl;
        }
        else 
        {
            std::cout << "responce error! =" << rbuf[0] << std::endl;
        }
    }
    
    std::cout << "wrote " << ss.str() << std::endl;
    return true;
}


bool Lv1Interface::getJointAngles(double *ang)
{
        
//     for (int i = 0; i < LV1_DOF; i++)
//     {
//         ang[i] = angles[i];
//     }
        char buf[64] = {0};
        if (write(fd, "g", 1) == 1)
        {
                usleep(50 * 1000);
                int ret;
                ret = read(fd, buf, 64);
                if ( ret > 0)
                {
                        printf("[%d] %s\n", ret , buf);
                        
                        sscanf(buf, "%lf %lf %lf %lf %lf", &ang[0], &ang[1], &ang[2], &ang[3], &ang[4]);
                        for (int i = 0; i < LV1_DOF; i++)
                        {
                                std::cout << ang[i] << std::endl;
                                ang[i] = ((ang[i] * 270.0) / 180.0) - offset[i] - 135;
                                //std::cout << ang[i] << std::endl;
                        }
                }
                else
                {
                        std::cerr << "read error" << std::endl;
                }
        }
        
    return true;
}


Lv1Interface::~Lv1Interface()
{
    // 設定を元に戻す
    tcsetattr(fd, TCSANOW, &oldtio);
    close(fd);
    std::cout << "closed device" << std::endl;
}


namespace
{
double kamae[LV1_DOF] = {-50, 0, 0, 0, 0};
double open2[LV1_DOF] = {90, -75, 100, 0, 0};
double phold[LV1_DOF] = {90, 0, 100, 0, 0};
double hold[LV1_DOF] = {95, 0, 0, 0, 0};
double up[LV1_DOF] = {90, -30, -20, 0, 0};
double forward[LV1_DOF] = {-70, 0, 0, 32, 0};
double sosogi[LV1_DOF] = {-70, -90, 0, 32, 0};
double modosu[LV1_DOF] = {-75, 0, 0, 32, 0};
double serve[LV1_DOF] = {-50, 0, 0, 32, 0};

double kamae_back[LV1_DOF] = {-50, 0, 0, 32, 0};
double phold_back[LV1_DOF] = {93, 0, 100, 32, 0};
double hold_back[LV1_DOF] = {93, 0, 0, 32, 0};
#if 0
double migi[LV1_DOF] = {-80, 0, 0, -30, 0};
double hidari[LV1_DOF] = {-30, 0, 0, 30, 0};
double tojiru[LV1_DOF] = {-80, 0, 0, 30, 0};
double hiraku[LV1_DOF] = {-30, 0, 0, -30, 0};
#endif
// double hold_back[LV1_DOF] = {90, 0, 0, 0, 0};
}

double GetTrayAngleByID(int tray_id)
{
    double angle = 0;

    switch(tray_id)
    {
    case 0:
            angle = 0;
            break;
    case 1:
            angle = 100;
            break;
    case 2:
            angle = -125;
            break;
    default:
            std::cerr << "invalid tray id" << std::endl;
            break;
    }
    return angle;
}

void SetJointAnglesWithTray(Lv1Interface &lv, double *pose, double time, int tray)
{
    pose[4] = GetTrayAngleByID(tray);
    lv.setJointAngles(pose, time);
}

void SetJointAnglesWithoutTray(Lv1Interface &lv, double *pose, double time)
{
    double state[LV1_DOF];
    lv.getJointAngles(state);
    pose[4] = state[4];
    lv.setJointAngles(pose, time);
}

void Sosogi(Lv1Interface &lv, int tray)
{
    SetJointAnglesWithTray(lv, kamae_back, 3.0, tray);
    
    SetJointAnglesWithTray(lv, open2, 3.0, tray);
    SetJointAnglesWithTray(lv, phold, 1.0, tray);
    SetJointAnglesWithTray(lv, hold, 1.0, tray);
    sleep(1);
    SetJointAnglesWithTray(lv, up, 1.0, tray);
    SetJointAnglesWithTray(lv, forward, 3.0, tray);
    SetJointAnglesWithTray(lv, sosogi, 2.0, tray);
    sleep(4);
    SetJointAnglesWithTray(lv, modosu, 1.0, tray);
    sleep(1);
    SetJointAnglesWithTray(lv, serve, 3.0, tray);
}

void SosogiBack(Lv1Interface &lv, int tray)
{
    SetJointAnglesWithTray(lv, kamae, 2.0, tray);
    SetJointAnglesWithTray(lv, up, 4.0, tray);
    SetJointAnglesWithTray(lv, hold_back, 2.0, tray);
    SetJointAnglesWithTray(lv, phold_back, 2.0, tray);
    SetJointAnglesWithTray(lv, open2, 3.0, tray);
    SetJointAnglesWithTray(lv, kamae, 3.0, tray);
}
