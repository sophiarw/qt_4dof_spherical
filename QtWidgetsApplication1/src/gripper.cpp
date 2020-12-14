#include "gripper.h"
#include "Windows.h"


gripper::gripper(void){	//:  pThumb(fingers::thumb), pIndex(fingers::index)


	// exoskeleton available for connection
	m_gripperAvailable = true;
	m_gripperReady = false;
	m_error = false;
	m_errMessage = "";
	
	// kinematic variables
	m_t = 0;
	m_thZero = 	{ PI / 2.0, PI / 2.0, PI / 2.0, PI / 2.0, 0.0 }; //{ 0.8796 , 2.2620, 0.8796 , 2.2620, 0 };	
	m_clk = new cPrecisionClock();
	m_th = m_thZero;
	m_thDes = m_thZero;
	m_thErr = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdot = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdotDes = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thdotErr = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_thErrInt = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	m_T = { 0.0, 0.0, 0.0, 0.0, 0.0 };
	gripperLength = 0.05;		// [m]

	//set up two pantographs:
	if (pThumb.m_pantographReady) { pThumb.m_finger = fingers::thumb; }
	if (pIndex.m_pantographReady) { pIndex.m_finger = fingers::index; }

	// set up filter
	lpfGrip.setCutOffFrequency(cutOffFreqGrip);

}

gripper::~gripper(void){
	// disconnect from S826 and free dynamically allocated memory
	if (disconnect()) {
		delete m_clk;
	}
}


bool gripper::connect()
	{
		// check if gripper is available/has been opened already
		if (!m_gripperAvailable) return(C_ERROR);
		if (m_gripperReady)      return(C_ERROR);

		// connect to S826
		bool success = connectToS826();
		if (!success) return(C_ERROR);

		// initialize encoders
		for (int i = 0; i < NUM_ENC; i++) {
			success = initEncod((uint)i);
			if (!success) return(C_ERROR);
		}

		// initialize motors
		for (int i = 0; i < NUM_MTR; i++) {
			success = initMotor((uint)i);
			if (!success) return(C_ERROR);
		}

		m_clk->start();
		m_gripperReady = true;
		return(C_SUCCESS);
	}


// receive force and torque information in the gripper's local coordinate frame
void gripper::setForcesAndTorques( cVector3d a_thumbForce, cVector3d a_fingerForce) {

	// in local coordinates (x toward wrist, y toward back of hand, z up)
	m_thumbForce = a_thumbForce;		// will only use x and z components (tangent to fingerpad)
	m_fingerForce = a_fingerForce;		// will only use x and z components (tangent to fingerpad)

	// TESTING
	//double period = 2.0;
	//double omega = 2 * PI / period;
	//m_thumbForce.z(0.9*sin(omega*m_clk->getCurrentTimeSeconds()));
	//m_fingerForce.z(0.9*sin(omega*m_clk->getCurrentTimeSeconds()));
	//m_thumbForce.x(0.0);
	//m_fingerForce.x(0.0);

	// calculate grip force torque
	if (m_thumbForce.y() < 0.0 && m_fingerForce.y() > 0.0) {
		m_gripForce = -m_thumbForce.y() + m_fingerForce.y();
	}
	else {
		m_gripForce = 0.0;
	}

	// add constant offset to contact users fingers
	m_gripForce += 4.0; //0.2
	m_T[4] = m_gripForce*gripperLength;

	// calculate desired position for each pantograph in pantograph class and solve for thDes with inverse kinematics
	pThumb.setPos(m_thumbForce);
	pIndex.setPos(m_fingerForce);
	//cout << "Pantographs:  " << pIndex.m_posDes.x() -pIndex.centerPoint.x() << ",  " << pIndex.m_posDes.z() - pIndex.centerPoint.z() << ", " << pThumb.m_posDes.x() - pThumb.centerPoint.x() << ", " << pThumb.m_posDes.z() - pThumb.centerPoint.z() << endl;

	// calculate motor angle commands 
	m_thDes[0] = pIndex.m_thDes.x();
	m_thDes[1] = pIndex.m_thDes.y();
	m_thDes[2] = pThumb.m_thDes.x();
	m_thDes[3] = pThumb.m_thDes.y();


	if (!hapticsOn) {
		disableCtrl();
	}

	// DEBUG 
	if (0) {

		//cout << m_thumbForce << "   " << m_fingerForce << "    ";
		cout << pThumb.m_posDes << ",    " << pIndex.m_posDes << ",  ";

		//m_gripperLock.acquire();
		//cout << m_th[0] * 180 / PI << " " << m_th[1] * 180 / PI << " " << m_th[2] * 180 / PI << " " << m_th[3] * 180 / PI << endl;

		//cout << "Index Forces: " << m_fingerForce.x() << "   " << -(m_fingerForce.z()) << "     ";
		//cout << "Index DesiredPosition: " << pIndex.m_posDes.x() << ", " << pIndex.m_posDes.z() << "     ";

		//cout << "Index Desired Angles: " 
			cout << cRadToDeg(m_thDes[0]) << ", " << cRadToDeg(m_thDes[1]) << ",     ";
		//cout << "Index Actual Angles: " 
			cout << cRadToDeg(m_th[0]) << ", " << cRadToDeg(m_th[1]) << ",     ";

		//cout << "Thumb DesiredAngles: " << cRadToDeg(m_thDes[2]) << ", " << cRadToDeg(m_thDes[3]) << "     ";
		//cout << "   Gripper Force: " << m_gripForce << endl;
		cout << endl;
		//m_gripperLock.release();
	}
}


void gripper::getState()
{
	// declare "memory" variables for calculations
	tLast = m_t;
	vector<double> thLast = m_th;
	vector<double> thdotLast = m_thdot;
	vector<double> thErrIntLast = m_thErrInt;

	// get current joint positions/errors
	for (int i = 0; i < NUM_MTR; i++) {
		m_th[i] = getAngle(i) + m_thZero[i];
	}
	//cout << m_th[1] * 180 / M_PI << ",   " << m_th[0]*180/M_PI << endl;

	for (int i = 0; i < NUM_MTR; i++) {
		m_thErr[i] = m_thDes[i] - m_th[i]; //angleDiff(m_thDes[i], m_th[i]);
	}
	
	// calculate velocities/errors
	m_t = m_clk->getCurrentTimeSeconds();
	for (int i = 0; i < NUM_MTR; i++) {
		m_thdot[i] = V_FILT*(angleDiff(m_th[i], thLast[i]) / (m_t - tLast)) + (1 - V_FILT)*thdotLast[i];
		m_thdotErr[i] = m_thdotDes[i] - m_thdot[i];

		// integrate position error
		m_thErrInt[i] = thErrIntLast[i] + m_thErr[i]*(m_t - tLast);
	}

	// clamp integrated error
	for (int i = 0; i < NUM_ENC; i++) {
		if (fabs(m_thErrInt[i]) > INT_CLMP) {
			m_thErrInt[i] = (m_thErrInt[i] / fabs(m_thErrInt[i]))*INT_CLMP;
		}
	}
}


void gripper::motorLoop(void)
{
	if (m_gripperReady) {
		getState();
		// calculate error
		for (int i = 0; i < NUM_MTR - 1; i++) {  // one joint at a time, only for pantograph motors
			m_T[i] = m_Kp[i] * m_thErr[i] + m_Kd[i] * m_thdotErr[i] + m_Ki[i] * m_thErrInt[i];
		}
		//DEBUG
		//cout << cRadToDeg(m_th[0]) << "   " << cRadToDeg(m_th[1]) << "           " << m_T[0] << "   " << m_T[1] <<  "  " << endl;

		for (int i = 0; i < NUM_MTR; i++) {
			setTorque(i, m_T[i]);
		}
	}
}

bool gripper::disableCtrl(void) {

	for (int i = 0; i < NUM_MTR; i++) {
		setVolts(i, 0.0);
	}

	//Sleep(1000);
	return(C_SUCCESS);
}

bool gripper::disconnect(void) {
	// check that exoskeleton is open
	if (!m_gripperReady) return(C_ERROR);

	// set motor torques to zero and disconnect from S826
	disableCtrl();
	cout << "Setting all Voltages to 0 V" << endl;
	Sleep(1000);
	disconnectFromS826();

	m_clk->stop();
	m_gripperReady = false;
	return(C_SUCCESS);
}


void gripper::getPantographPos(double& X1, double& Y1, double& X2, double& Y2) {
	X1 = pIndex.m_posDes.x() - pIndex.centerPoint.x();
	Y1 = pIndex.m_posDes.z() - pIndex.centerPoint.z();
	X2 = pThumb.m_posDes.x() - pIndex.centerPoint.x();
	Y2 = pThumb.m_posDes.z() - pIndex.centerPoint.z();
}