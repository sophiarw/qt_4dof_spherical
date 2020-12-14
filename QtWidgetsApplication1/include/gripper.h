#pragma once
#include "chai3d.h"
#include "pantograph.h"
#include "LowPassFilter.hpp"


#include <cmath>
#include <array>
#include <cstdlib>

using namespace chai3d;
using namespace std;

#define NUM_ENC 5	// nunber of encoders 2 on each finger and gripper motor = 5
#define NUM_MTR 5 

enum { indexDist, indexProx, thumbProx, thumbDist, gripMotor };

////------------------------------------------------------------------------------
//// ERROR CONSTANTS
//const bool C_ERROR = false;			//! Function returns with an error.
//const bool C_SUCCESS = true;		//! Function returns successfully.
////------------------------------------------------------------------------------


class gripper : public cGenericHapticDevice
{

private:

public:
	bool hapticsOn = true;

	pantograph pThumb;
	pantograph pIndex;

	cVector3d m_thumbForce;
	cVector3d m_fingerForce;
	double m_gripForce;
	LowPassFilter lpfGrip;
	double cutOffFreqGrip = 80 / (2 * 3.14159);


	bool m_error;                           // TRUE = problem with device while running
	std::string m_errMessage;               // error message
	double m_t;                             // current time [sec]
	chai3d::cPrecisionClock* m_clk;         // pointer to clock for computing velocity
	double tLast;

	bool m_gripperAvailable;
	bool m_gripperReady;
	chai3d::cMutex m_gripperLock;

	vector<double> m_thZero;                  // zero angles for motor-angle measurement [counts, in motor space]

	vector<double> m_th;				//	motor angles [rad]
	vector<double> m_thDes;			// desired motor angles [rad]
	vector<double> m_thErr;              // joint-angle error [rad]	
	vector<double> m_thdot;              // current joint velocities [rad/s]
	vector<double> m_thdotDes;           // desired joint velocities [rad/s]
	vector<double> m_thdotErr;           // joint-velocity error [rad/s]
	vector<double> m_thErrInt;           // integrated joint angle error [rad*s]
	vector<double> m_T;		// motor torques to command 
	//const vector<double> m_Kp = { 0.3, 0.3, 0.3, 0.3, 10 };  //{ 50, 50, 50, 50, 10 }; // [N/rad]
	//const vector<double> m_Kd = { 0.002, 0.002, 0.002, 0.002, 1 }; //{ 0.00001, .00001, 0.00001, .00001, 0.1 };
	//const vector<double> m_Ki = {0.0001, 0.0001, 0.0001, 0.0001, 1}; //{ 0.01, 0.01, 0.01, 0.01, 0.01 };
	const vector<double> m_Kp = { 0.2, 0.2, 0.2, 0.2, 1.0 };  //{ 50, 50, 50, 50, 10 }; // [N/rad]			// TO DO: TUNE GAINS
	const vector<double> m_Kd = { 0.004, 0.004, 0.004, 0.004, 0.0 }; //{ 0.00001, .00001, 0.00001, .00001, 0.1 };
	const vector<double> m_Ki = { 0.000, 0.000, 0.000, 0.000, 0.000 }; //{ 0.01, 0.01, 0.01, 0.01, 0.01 };


	gripper(); // : pThumb(fingers::thumb), pIndex(fingers::index) { }
	~gripper();
	bool connect();
	bool disconnect();
	bool disableCtrl();
	void getState();
	void getPantographPos(double& X1, double& Y1, double& X2, double& Y2);
	void setForcesAndTorques(cVector3d a_thumbForce, cVector3d a_fingerForce);
	void motorLoop(void);
	double gripperLength;
private:

};