#ifndef __LV1_H__
#define __LV1_H__

#include <termios.h>
#include <string>
#include <stdlib.h>
#include <iostream>

#define LV1_DOF (5)

class Lv1Interface {
public:
  Lv1Interface();
  ~Lv1Interface();
  bool Open(const std::string &devName);
  void setSerialPort();
  void setInitialAngles();
  bool angleLimitFilter(double *ang, int i);
  bool checkAngleLimits(double *ang);
  bool setJointAngles(double *ang, double time);
  bool setJointAnglesInternal(double *ang);
  bool getJointAngles(double *ang);
  
private:
  int fd;
  int angles[LV1_DOF];
  int servo[LV1_DOF];
  int offset[LV1_DOF];
  int hiAngleLimits[LV1_DOF];
  int loAngleLimits[LV1_DOF];
  struct termios oldtio;
};

double GetTrayAngleByID(int tray_id);
void SetJointAnglesWithTray(Lv1Interface &lv, double *pose, double time, int tray);
void SetJointAnglesWithoutTray(Lv1Interface &lv, double *pose, double time);
void Sosogi(Lv1Interface &lv, int tray);
void SosogiBack(Lv1Interface &lv, int tray);

#endif
