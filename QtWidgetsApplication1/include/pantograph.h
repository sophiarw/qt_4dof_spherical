#pragma once
#include "chai3d.h"
#include "motorcontrol.h"
#include <cmath>
#include <array>
#include <algorithm>
#include "LowPassFilter.hpp"

#define V_FILT         0.5      // weight for velocity filtering
#define INT_CLMP       0.1      // maximum allowed integrated error
#define PI  3.14159
#define MAX_STRETCH 4.0			// maximum radius of stretch from centerpoint

enum fingers {
	thumb,
	index
};


class pantograph {

public:

	bool m_error;                           // TRUE = problem with exo while running
	std::string m_errMessage;               // error message
	double m_t;                             // current time [sec]
	chai3d::cPrecisionClock* m_clk;         // pointer to clock for computing velocity


	// arm lengths: upper left, bottom left, bottom right, upper right, top bar
	double len[5] = { 10, 13, 13, 10, 10 };		// small motors: { 10, 13, 13, 10, 10};		
	double center[2] = {0.8796, 2.2620};

	double m_finger;		// index or thumb (mirrored x position values)

	chai3d::cVector3d m_th;                 // current joint angles [rad]
	chai3d::cVector3d m_thDes;              // desired joint angles [rad]
	chai3d::cVector3d m_posDes;             // desired end-effector position [mm]

	chai3d::cVector3d centerPoint;

	pantograph();
	~pantograph();

	void setPos(chai3d::cVector3d a_force); // double a_x, double a_y);

	bool m_pantographAvailable;            // TRUE = device instance has been created
	bool m_pantographReady;                // TRUE = connection to device successful

protected:
	double k_skin = 1.58;		// [N/mm] scale force to skin displacement
	void inverseKinematics();
	double angleDiff(double a_thA, double a_thB);
	chai3d::cVector3d vecDiff(chai3d::cVector3d a_vecA, chai3d::cVector3d a_vecB);

	// filter
	LowPassFilter lpfX;
	LowPassFilter lpfZ;
	double cutOffFreqFinger = 500 / (2 * 3.14159);
	chai3d::cPrecisionClock pantographTimer;


};
