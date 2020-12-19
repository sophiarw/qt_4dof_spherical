#include "c4DOFGripper.h"
#include "Windows.h"


c4DOFGripper::c4DOFGripper(int location) {
	
	// devices available for connection
	m_c4DOFGripperAvailable = true;
	m_c4DOFGripperReady = false;
	m_error = false;
	m_errMessage = "";

	// kinematic variables
	m_t = 0;
	m_clk = new cPrecisionClock();
	m_th = { 0.0, 0.0, 0.0, 0.0 };
	m_thDes = { 0.0, 0.0, 0.0, 0.0 };
	m_thErr = { 0.0, 0.0, 0.0, 0.0 };
	m_thdot = { 0.0, 0.0, 0.0, 0.0 };
	m_thdotDes = { 0.0, 0.0, 0.0, 0.0 };
	m_thdotErr = { 0.0, 0.0, 0.0, 0.0 };
	m_thErrInt = { 0.0, 0.0, 0.0, 0.0 };
	m_T = { 0.0, 0.0, 0.0, 0.0 };
	angle = { 0.0, 0.0, 0.0, 0.0 };
	gripperLength = 0.05;		// [m]

								//set up two pantographs:
	//if (pThumb.m_c4DOFGripperReady) { pThumb.m_finger = fingers::thumb; }
	if (device.m_c4DOFDeviceReady) { device.m_location = location; }

	// set up filter
	lpfGrip.setCutOffFrequency(cutOffFreqGrip);

	filename = "motor_inputs_outputs.csv";
	file.open(filename);
	file << "m_theErr[0], m_theErr[1], m_theErr[2], m_theErr[3], m_thdotErr[0], m_thdotErr[1], m_thdotErr[2], m_thdotErr[3], m_T[0], m_T[1], m_T[2], m_T[3]" << endl;
}

c4DOFGripper::~c4DOFGripper() {
	// disconnect from S826 and free dynamically allocated memory
	if (disconnect()) {
		delete m_clk;
	}
	if (file.is_open()) {
		file.close();
	}
}


bool c4DOFGripper::connect()
{
	// check if gripper is available/has been opened already
	if (!m_c4DOFGripperAvailable) return(C_ERROR);
	if (m_c4DOFGripperReady)      return(C_ERROR);

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
	m_c4DOFGripperReady = true;
	return(C_SUCCESS);
}

// receive force and torque information in the gripper's local coordinate frame
// I need to write this function based on Jake's code???
void c4DOFGripper::setForcesAndTorques(cVector3d a_force, cVector3d a_torque) {

	if (0) {
		cout << "forces from model:" << a_force.x() << ", " << a_force.y() << ", " << a_force.z() << ", " << a_torque.z() << endl;
	}
	m_force = a_force;		// will only use x and z components (tangent to fingerpad)
	m_torque = a_torque; //only need the torsion around the z axis

	Eigen::Vector4d force = Eigen::Vector4d(m_force.x(), m_force.y(), m_force.z(), m_torque.z());

	//cout << "The torque is: " << m_torque.z() << endl;
	// calculate desired position for each pantograph in pantograph class and solve for thDes with inverse kinematics
	//pThumb.setPos(m_thumbForce);

	//convert forces to positions
	device.setPos(force);
	
	// calculate motor angle commands 
	m_thDes[0] = device.m_thDes.x();
	m_thDes[1] = device.m_thDes.y();
	m_thDes[2] = device.m_thDes.z();
	m_thDes[3] = device.m_thDes.w();


	if (!hapticsOn) {
		disableCtrl();
	}

	// DEBUG 
	if (0) {

		cout << "c4DOFDevice:  " << device.m_posDes.x() - device.centerPoint.x() << ",  " << device.m_posDes.z() - device.centerPoint.z() << "," << endl;



		//cout << "Index Desired Angles: " 
		cout << cRadToDeg(m_thDes[0]) << ", " << cRadToDeg(m_thDes[1]) << ",     ";
		//cout << "Index Actual Angles: " 
		cout << cRadToDeg(m_th[0]) << ", " << cRadToDeg(m_th[1]) << ",     ";
		cout << endl;
	}
}



void c4DOFGripper::getState()
{
	// declare "memory" variables for calculations
	tLast = m_t;
	vector<double> thLast = m_th;
	vector<double> thdotLast = m_thdot;
	vector<double> thErrIntLast = m_thErrInt;

	// get current joint positions/errors
	for (int i = 0; i < NUM_MTR; i++) {
		angle[i] = getAngle(i);
		m_th[i] = angle[i];
	}


	for (int i = 0; i < NUM_MTR; i++) {
		m_thErr[i] = m_thDes[i] - m_th[i]; //angleDiff(m_thDes[i], m_th[i]);
		
	}


	// calculate velocities/errors
	m_t = m_clk->getCurrentTimeSeconds();
	for (int i = 0; i < NUM_MTR; i++) {
		m_thdot[i] = V_FILT*(angleDiff(m_th[i], thLast[i]) / (m_t - tLast)) + (1 - V_FILT)*thdotLast[i];
		m_thdotErr[i] = m_thdotDes[i] - m_thdot[i];

		// integrate position error
		m_thErrInt[i] = thErrIntLast[i] + m_thErr[i] * (m_t - tLast);
	}

	// clamp integrated error
	for (int i = 0; i < NUM_ENC; i++) {
		if (fabs(m_thErrInt[i]) > INT_CLMP) {
			m_thErrInt[i] = (m_thErrInt[i] / fabs(m_thErrInt[i]))*INT_CLMP;
		}
	}

	
}


void c4DOFGripper::motorLoop(void)
{
	if (m_c4DOFGripperReady) {
		getState();
		// calculate error
		for (int i = 0; i < NUM_MTR; i++) {  // one joint at a time
			m_T[i] = m_Kp[i] * m_thErr[i] + m_Kd[i] * m_thdotErr[i] + m_Ki[i] * m_thErrInt[i];
		}
		//DEBUG
		//cout << cRadToDeg(m_th[0]) << "   " << cRadToDeg(m_th[1]) << "           " << m_T[0] << "   " << m_T[1] <<  "  " << endl;
		//cout << "m_t: " << cRadToDeg(m_th[0]) << ",  m_T: " << m_T[0] << "  " << endl;
		//print motor inputs and outputs to file


		file << m_thErr[0] << ", " << m_thErr[1] << ", " << m_thErr[2] << ", " << m_thErr[3] << ", ";
		file << m_thdotErr[0] << ", " << m_thdotErr[1] << ", " << m_thdotErr[2] << ", " << m_thdotErr[3] << ", ";
		file << m_T[0] << ", " << m_T[1] << ", " << m_T[2] << ", " << m_T[3] << endl;
		//file << m_th[0] << ", " << m_th[0] << ", " << m_th[0] << ", " << m_th[0] << ", ";
		//file << angle[0] << ", " << angle[1] << ", " << angle[2] << ", " << angle[3] << ", ";
		//file << m_thDes[0] << ", " << m_thDes[1] << ", " << m_thDes[2] << ", " << m_thDes[3] << ", ";
		//file << m_thErrInt[0] << ", " << m_thErrInt[1] << ", " << m_thErrInt[2] << ", " << m_thErrInt[3] << ", ";
		
		for (int i = 0; i < NUM_MTR; i++) {
			setTorque(i, -m_T[i]);
		}
	}
}


bool c4DOFGripper::disableCtrl(void) {

	for (int i = 0; i < NUM_MTR; i++) {
		setCurrent(i, 0.0);
	}

	return(C_SUCCESS);
}


bool c4DOFGripper::disconnect(void) {
	// check that exoskeleton is open
	if (!m_c4DOFGripperReady) return(C_ERROR);

	// set motor torques to zero and disconnect from S826
	disableCtrl();
	cout << "Setting all Voltages to 0 V" << endl;
	Sleep(1000);
	disconnectFromS826();

	m_clk->stop();
	m_c4DOFGripperReady = false;
	return(C_SUCCESS);
}

void c4DOFGripper::getc4DOFGripperPos(Eigen::Ref<Eigen::Vector4d> a_Pos) {
	a_Pos[0] = device.m_posDes[0] - device.centerPoint[0];
	a_Pos[1] = device.m_posDes[1] - device.centerPoint[1];
	a_Pos[2] = device.m_posDes[2] - device.centerPoint[2];
	a_Pos[3] = device.m_posDes[3] - device.centerPoint[3];

}

void c4DOFGripper::getNeutralPos(Eigen::Ref<Eigen::Vector4d> a_neutralPos) {
	Eigen::Vector4d neutralPos;
	neutralPos = device.neutralPos;
	a_neutralPos.x() = neutralPos.x();
	a_neutralPos.y() = neutralPos.y();
	a_neutralPos.z() = neutralPos.z();
	a_neutralPos.w() = neutralPos.w();
}