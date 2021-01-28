#pragma once

#include "chai3d.h"
#include "trakSTAR.h"
#include "LowPassFilter.hpp"
#include "vec3LPF.h"


#define MAGTRACKER
#define NUM_TRACKERS 2

using namespace std;
using namespace chai3d;

class magTrackerThread
{
public:

	explicit magTrackerThread();
	~magTrackerThread();
	
	int trackerNum;
	bool trackingOn;

	// filtering
	float cutOffFreq = 50.0/(2*3.14159);	// (1.5* 2 * PI) Hz
	Vec3LowPassFilter lpf;

	void initMagTracker(int a_trackerNum);
	cTransform CheckTrackerPose();
	
	// tracker offset
	cVector3d posOffsetInBodyFrame = { 0.02, 0.0, -0.03 }; // yellow handle: (0.034, 0.0, -0.094); previous blue handle values (0.02, 0.0,-.09)


	// device, pair with haptics thread
	chai3d::cGenericHapticDevicePtr* m_chaiMagDevice;
	void pairWithHapticsThread(chai3d::cGenericHapticDevicePtr *a_chaiMagDevice);

	chai3d::cPrecisionClock runTimer;

	// magnetic tracker variables
	CSystem     ATC3DG; // a pointer to a single instance of the system class
	CSensor     *pSensor; // a pointer to an array of sensor objects
	CXmtr       *pXmtr; // a pointer to an array of transmitter objects
	CBoard      *pBoard; // a pointer to an array of board objects
	int         errorCode;
	int         sensorID;
	int         transmitterID;
	short       id;
	int         numberBytes;
	int         i;
	double      measFreq;
	DOUBLE_POSITION_MATRIX_TIME_STAMP_RECORD record;

};

