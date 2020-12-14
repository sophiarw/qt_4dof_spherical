#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include "826api.h"
#include <cmath>
#include <iostream>
#include <algorithm>

#define THRESH 0.00001

// important values for motors
// 0: index distal
// 1: index proximal
// 2: thumb proximal
// 3: thumb distal
// 4: grip force motor

	
const double gearRatio[4] = { 57, 57, 57, 57 }; // encoder resolution	
const double encoderLinesPerRev[4] = { 128,  128,  128,  128 }; // 128 * 4 = 512
const double amplifierResistance[6] = { 10.0, 10.0, 10.0, 10.0, 10.0, 10.0 }; // the resistance of the current amplifier
const double k_torq[4] = { 1.56 * 57 /1000, 1.56 * 57 / 1000,  1.56 * 57 / 1000, 1.56 * 57 / 1000 };// [N-m/A]// 0.524 mNm / 0.336A /1000 = 0.00156 Nm. Need to multiply by 57 //include gear ratio in k_torque


bool connectToS826();
void disconnectFromS826();
bool initMotor(uint channel);
bool initEncod(uint channel);
bool checkEncod(uint channel);
void setCurrent(uint channel, double V);
void setTorque(uint channel, double T);
int setCounts(uint channel, uint counts);
uint getCounts(uint channel);
double getAngle(uint channel);
double angleDiff(double a_thA, double a_thB);

using namespace std;

#endif // MOTORCONTROL_H
