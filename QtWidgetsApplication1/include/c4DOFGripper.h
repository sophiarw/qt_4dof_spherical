#pragma once
#include "chai3d.h"
#include "c4dofdevice.h"
#include "LowPassFilter.hpp"
#include <Eigen/Dense>
#include <fstream>

#include <cmath>
#include <array>
#include <cstdlib>

using namespace chai3d;
using namespace std;

// TO DO: Expand to two devices
#define NUM_ENC 4	// nunber of encoders 2 on each finger and gripper motor = 5
#define NUM_MTR 4

enum { motor1_loc1, motor1_loc2, motor1_loc3, motor1_loc4 };

class c4DOFGripper : public cGenericHapticDevice
{
private:

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	bool hapticsOn = true;

	c4DOFDevice device;

	cVector3d m_force;
	cVector3d m_torque;

	double m_gripForcel;
	LowPassFilter lpfGrip;
	double cutOffFreqGrip = 80 / (2 * 3.14159);

	bool m_error;                           // TRUE = problem with device while running
	std::string m_errMessage;               // error message
	double m_t;                             // current time [sec]
	chai3d::cPrecisionClock* m_clk;         // pointer to clock for computing velocity
	double tLast;

	bool m_c4DOFGripperAvailable;
	bool m_c4DOFGripperReady;
	chai3d::cMutex m_c4DOFGripperLock;

	//I decided to use vector<double> instead of Eigen::Vector4d because the vector is
	//suposed to include the values for all motots, even when there are 8
	vector<double> m_th;					// motor angles [rad]
	vector<double> m_thDes;					// desired motor angles [rad]
	vector<double> m_thErr;					// joint-angle error [rad]	
	vector<double> m_thdot;					// current joint velocities [rad/s]
	vector<double> m_thdotDes;				// desired joint velocities [rad/s]
	vector<double> m_thdotErr;				// joint-velocity error [rad/s]
	vector<double> m_thErrInt;				// integrated joint angle error [rad*s]
	vector<double> m_T;						// motor torques to command 
	vector<double> angle;					// angle calculated from encoder counts

											// TO DO: TUNE GAINS
	const vector<double> m_Kp = { 0.2, 0.2, 0.2, 0.2 }; // {1.0, 1.0, 1.0, 1.0 };		// [N/rad]			
	const vector<double> m_Kd = { 0.004, 0.004, 0.004, 0.004}; //0.02
	const vector<double> m_Ki = { 0.000, 0.000, 0.000, 0.000 }; 

	c4DOFGripper(int location);
	~c4DOFGripper();
	bool connect();
	bool disconnect();
	bool disableCtrl();
	void getState();
	void getc4DOFGripperPos(Eigen::Ref<Eigen::Vector4d>  a_Pos);
	void setForcesAndTorques(cVector3d a_force, cVector3d a_torque);
	void motorLoop();
	double gripperLength;

	ofstream file;
	string filename;




};