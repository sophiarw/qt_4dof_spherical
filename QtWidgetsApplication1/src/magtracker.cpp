#include "magtracker.h"

magTrackerThread::magTrackerThread()
{
	// set up filter
	//lpfX.setCutOffFrequency(cutOffFreq);
	//lpfY.setCutOffFrequency(cutOffFreq);
	//lpfZ.setCutOffFrequency(cutOffFreq);
	lpf.setCutOffFreq(cutOffFreq);
}

magTrackerThread::~magTrackerThread()
{
	// Turn off the transmitter before exiting
	// We turn off the transmitter by "selecting" a transmitter with an id of "-1"
	//
	int id = -1;
	errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
	if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);


	//  Free memory allocations before exiting
	//
	delete[] pSensor;
	delete[] pXmtr;
	trackingOn = false;
}

void magTrackerThread::pairWithHapticsThread(chai3d::cGenericHapticDevicePtr *a_chaiMagDevice) {
	m_chaiMagDevice = a_chaiMagDevice;

}

void magTrackerThread::initMagTracker(int a_trackerNum)
{
	trackerNum = a_trackerNum;
	trackingOn = true;
	//runTimer.setTimeoutPeriodSeconds(0.001);
	

#ifdef MAGTRACKER
	// initialize the magnetic tracker
	std::cout << "Initializing the ATC3DG system...\n" << std::endl;
	errorCode = InitializeBIRDSystem();
	if (errorCode == BIRD_ERROR_SUCCESS) {
		std::cout << "Initialized ATC3DG system\n" << std::endl;
	}

	// get configurations
	errorCode = GetBIRDSystemConfiguration(&ATC3DG.m_config);
	pSensor = new CSensor[ATC3DG.m_config.numberSensors];
// for (i = 0; i<ATC3DG.m_config.numberSensors; i++)
	for (i = 0; i<1; i++)	// always using first one
	{
		errorCode = GetSensorConfiguration(i, &pSensor[i].m_config);
		if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
		std::cout << "Got sensors configuration\n" << std::endl;
	}
	pXmtr = new CXmtr[ATC3DG.m_config.numberTransmitters];
	//for (i = 0; i<ATC3DG.m_config.numberTransmitters; i++)
	for (i = 0; i<1; i++)   // always using first one
	{
		errorCode = GetTransmitterConfiguration(i, &pXmtr[i].m_config);
		if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode);
		std::cout << "Got transmitters configuration\n" << std::endl;
	}

	measFreq = 80.0;
	// set parameters for recording
	SET_SYSTEM_PARAMETER(SELECT_TRANSMITTER, 0);
	SET_SYSTEM_PARAMETER(POWER_LINE_FREQUENCY, 60.0);
	SET_SYSTEM_PARAMETER(AGC_MODE, SENSOR_AGC_ONLY);
	SET_SYSTEM_PARAMETER(MEASUREMENT_RATE, measFreq);
	SET_SYSTEM_PARAMETER(MAXIMUM_RANGE, 72.0);
	SET_SYSTEM_PARAMETER(METRIC, true);

	for (sensorID = 0; sensorID<ATC3DG.m_config.numberSensors; sensorID++) {
		SET_SENSOR_PARAMETER(sensorID, DATA_FORMAT, DOUBLE_POSITION_MATRIX_TIME_STAMP);
		{
			// initialize a structure of angles
			DOUBLE_ANGLES_RECORD anglesRecord = { 0, 0, 0 };
			SET_SENSOR_PARAMETER(sensorID, ANGLE_ALIGN, anglesRecord);
		}
		SET_SENSOR_PARAMETER(sensorID, HEMISPHERE, FRONT);
		SET_SENSOR_PARAMETER(sensorID, FILTER_AC_WIDE_NOTCH, false);
		SET_SENSOR_PARAMETER(sensorID, FILTER_AC_NARROW_NOTCH, false);
		SET_SENSOR_PARAMETER(sensorID, FILTER_LARGE_CHANGE, false)
		SET_SENSOR_PARAMETER(sensorID, FILTER_DC_ADAPTIVE, 0.5);		// increase value to increase filtering
	}

	transmitterID = 0;
	{
		// initialize a structure of angles
		DOUBLE_ANGLES_RECORD anglesRecord = { 0, 0, 0 };
		SET_TRANSMITTER_PARAMETER(transmitterID, REFERENCE_FRAME, anglesRecord);
	}
	SET_TRANSMITTER_PARAMETER(transmitterID, XYZ_REFERENCE_FRAME, false);
#endif

	runTimer.start();
}


chai3d::cTransform magTrackerThread::CheckTrackerPose()
{
	// rotation matrix from magnetic tracker world (x axis facing user, z down) 
	// to chai3d world (x axis facing user, z up)
	chai3d::cMatrix3d MTtoCHAI(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);
	chai3d::cMatrix3d transformedMatrix;
	chai3d::cVector3d posOffsetInWorldFrame;

#ifdef MAGTRACKER
	// change to <= 1 if we want to use both trackers
	for (int tracker = 0; tracker < NUM_TRACKERS; tracker++)
	{
		double posScale = 1000.0;	// divide by 1000 to get to meters (?)
		double depthOffset = 70; // 130;
		double heightOffset = -30;
		double horizontalOffset = 0; // -100;

		chai3d::cTransform returnTransform;
		chai3d::cVector3d returnVec;
		chai3d::cMatrix3d returnMatrix;
		chai3d::cMatrix3d initialMatrix;
		chai3d::cVector3d initialVec;
		//double x, y, z;

		// test the reading of the magnetic tracker
		errorCode = GetAsynchronousRecord(tracker, &record, sizeof(record));
		if (errorCode != BIRD_ERROR_SUCCESS) { errorHandler(errorCode); }
		// get the status of the last data record
		// only report the data if everything is okay

		double newX = (record.x - depthOffset) / posScale;
		double newY = (record.y - horizontalOffset) / posScale;
		double newZ = (record.z - heightOffset) / posScale;
		float deltaT = runTimer.getCurrentTimeSeconds();
		cVector3d newXYZ = lpf.update(cVector3d(newX, newY, newZ), deltaT);

		//x = lpfX.update((record.x - depthOffset) / posScale, deltaT);
		//y = lpfY.update((record.y - horizontalOffset) / posScale, deltaT);
		//z = lpfZ.update((record.z - heightOffset) / posScale, deltaT);

		runTimer.reset();

		initialVec.set(newXYZ.x(), -newXYZ.y(), -newXYZ.z());		// transform from magtracker frame to chai3d frame
			
		initialMatrix.set(record.s[0][0], record.s[0][1], record.s[0][2],
			record.s[1][0], record.s[1][1], record.s[1][2],
			record.s[2][0], record.s[2][1], record.s[2][2]);
		initialMatrix.trans();
		MTtoCHAI.mulr(initialMatrix, transformedMatrix);
		transformedMatrix.rotateAboutLocalAxisDeg(0, 1, 0, 70); // for mag tracker tilted in handle	
		transformedMatrix.rotateAboutLocalAxisDeg(0, 0, 1, 180); // for mag tracker chord facing us instead of base box

		// account for magnetic tracker located at bottom of handle 
		transformedMatrix.mulr(-posOffsetInBodyFrame, posOffsetInWorldFrame);	
		initialVec.add(posOffsetInWorldFrame);

		// Pass information to chaiDevice for use in the haptics thread
		returnTransform.set(initialVec, transformedMatrix);

		return returnTransform;

	}

#endif
	
	//if no mag tracker
	chai3d::cTransform defaultTransform;
	return defaultTransform;
}
