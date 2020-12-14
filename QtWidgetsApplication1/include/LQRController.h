#pragma once
#include "vec3LPF.h"
#include "chai3d.h"
#include <fstream>
#include <iostream>

using namespace std;
using namespace chai3d;
using namespace Eigen;

class LQRController {

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	LQRController(void);
	void updateAll(cVector3d a_pos, cVector3d a_euler, double a_xErr, double a_zErr, double a_rollErr, double a_yawErr );
	Eigen::Vector4f getCommand();
	Eigen::Matrix<float,4,12, Eigen::DontAlign> getGains();
	Eigen::Matrix<float, 12, 1, Eigen::DontAlign> getState();


protected:
	// state variables
	double m_xErr, m_zErr;
	double m_yawErr, m_rollErr;
	cVector3d m_pos, m_vel;			// world frame position and velocity
	cVector3d m_euler, m_rotvel;	// world frame rotation and angular rates
	cVector3d m_uvw, m_uvwDot;		// body-frame velocities and accelerations
	cVector3d m_pqr, m_pqrDot;		// body-frame angular velocities and accelerations
	Eigen::Matrix<float,12,1,Eigen::DontAlign> state;

	// low pass filter for velocities and rotations (which isn't filtered in magtracker file)
	float cutOffFreq = 10.0 / (2 * 3.14159);	// (1.5* 2 * PI) Hz
	Vec3LowPassFilter yprFilter;
	Vec3LowPassFilter rotVelFilter;
	Vec3LowPassFilter linVelFilter;
	
												// clock 
	float last_time, deltaT;
	cPrecisionClock* m_clk;

	// helper functions
	void getUVW();		// calculate m_uvw and m_uvwDot
	void getPQR();		// calculate m_pqr and m_pqrDot

						// update functions
	void setPos(cVector3d a_pos);
	void setRot(cVector3d a_euler);
	void setError(double a_xErr, double a_zErr, double a_yawErr, double a_rollErr);

	// control law functions
	void readGainMatrix(std::string file);
	void calculateCommand();

	// control law variables
	Eigen::Vector4f u;
	ifstream gainFile;
	Eigen::Matrix<float, 4, 12, Eigen::DontAlign> K;


};
