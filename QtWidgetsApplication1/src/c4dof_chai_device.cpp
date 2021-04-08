/*
\author <http://wwww.chai3d.org>
\author Sophia Williams, Stanford University
\version 3.2.0
*/
#include "system/CGlobals.h"
#include "c4dof_chai_device.h"

//------------------------------------------------------------------------------
#if defined(C_ENABLE_CUSTOM_DEVICE_SUPPORT)
//------------------------------------------------------------------------------

namespace chai3d {

	//--------------------------------------------------------------------
	// CONSTRUCTOR of CHAIDEVICE
	//--------------------------------------------------------------------

c4dofChaiDevice::c4dofChaiDevice(unsigned int a_deviceNumber):D(a_deviceNumber)
{

	location = a_deviceNumber; //assign device/location number, so the thumb or index finger can be identified

	m_deviceReady = false;

	//specifications of device

	// haptic device model (see file "CGenericHapticDevice.h")
	m_specifications.m_model = C_HAPTIC_DEVICE_CUSTOM;

	// name of the device manufacturer, research lab, university.
	m_specifications.m_manufacturerName = "Stanford CHARM Lab";

	// name of your device
	m_specifications.m_modelName = "4dof Parallel Mechanism";

	//--------------------------------------------------------------------------						/// ------------------------------- TO DO ----------------------------------- 
	// CHARACTERISTICS: (The following values must be positive or equal to zero)
	//--------------------------------------------------------------------------

	// the maximum force [N] the device can produce along the x,y,z axis.
	m_specifications.m_maxLinearForce = 5.0;     // [N]

													// the maximum amount of torque your device can provide arround its
													// rotation degrees of freedom.
	m_specifications.m_maxAngularTorque = 0.2;     // [N*m]


													// the maximum amount of torque which can be provided by your gripper
	m_specifications.m_maxGripperForce = 3.0;     // [N]

													// the maximum closed loop linear stiffness in [N/m] along the x,y,z axis
	m_specifications.m_maxLinearStiffness = 1000.0; // [N/m]

													// the maximum amount of angular stiffness
	m_specifications.m_maxAngularStiffness = 1.0;    // [N*m/Rad]

														// the maximum amount of stiffness supported by the gripper
	m_specifications.m_maxGripperLinearStiffness = 1000;   // [N*m]

															// the radius of the physical workspace of the device (x,y,z axis)
	m_specifications.m_workspaceRadius = 0.2;     // [m]

													// the maximum opening angle of the gripper
	m_specifications.m_gripperMaxAngleRad = cDegToRad(30);

	////////////////////////////////////////////////////////////////////////////
	/*
	DAMPING PROPERTIES:

	Start with small values as damping terms can be high;y sensitive to
	the quality of your velocity signal and the spatial resolution of your
	device. Try gradually increasing the values by using example "01-devices"
	and by enabling viscosity with key command "2".
	*/
	////////////////////////////////////////////////////////////////////////////

	// Maximum recommended linear damping factor Kv
	m_specifications.m_maxLinearDamping = 20.0;   // [N/(m/s)]

													//! Maximum recommended angular damping factor Kv (if actuated torques are available)
	m_specifications.m_maxAngularDamping = 0.0;	  // [N*m/(Rad/s)]

													//! Maximum recommended angular damping factor Kv for the force gripper. (if actuated gripper is available)
	m_specifications.m_maxGripperAngularDamping = 0.0; // [N*m/(Rad/s)]


	//--------------------------------------------------------------------------
	// CHARACTERISTICS: (The following are of boolean type: (true or false)
	//--------------------------------------------------------------------------

	// does your device provide sensed position (x,y,z axis)?
	m_specifications.m_sensedPosition = true;

	// does your device provide sensed rotations (i.e stylus)?
	m_specifications.m_sensedRotation = true;

	// does your device provide a gripper which can be sensed?
	m_specifications.m_sensedGripper = true;

	// is you device actuated on the translation degrees of freedom?
	m_specifications.m_actuatedPosition = true;

	// is your device actuated on the rotation degrees of freedom?
	m_specifications.m_actuatedRotation = true;

	// is the gripper of your device actuated?
	m_specifications.m_actuatedGripper = true;

	// can the device be used with the left hand?
	m_specifications.m_leftHand = true;

	// can the device be used with the right hand?
	m_specifications.m_rightHand = true;

	scaleFactor = 2;
	////////////////////////////////////////////////////////////////////////////
	//Code which tells the application if your
	//device is actually connected to your computer and can be accessed.	
	////////////////////////////////////////////////////////////////////////////

	m_deviceAvailable = true;		// my code making device available

	//assign neutralPos
	neutralPos = { 0.0, 0.0, -20.0, 0.0 };
	newNeutralPos = false;

	//assign torsion state
	newTorsionState = false;
	torsionState = true;

}


//----------------------------------------------------------------------------
// Destructor of c4dofChaiDevice
//----------------------------------------------------------------------------
c4dofChaiDevice::~c4dofChaiDevice()
{
	if (m_deviceReady) { close(); }
}

//----------------------------------------------------------------------------
// Open connection to your device
// \return __true__ if successful, __false__ otherwise.
//----------------------------------------------------------------------------
bool c4dofChaiDevice::open()
{
	// check if the system is available
	if (!m_deviceAvailable) return (C_ERROR);

	MT.initMagTracker(location);

	//if system is already opened then return
	if (m_deviceReady) return (C_ERROR);

	bool result = C_ERROR;

	// the following does not exist but could be implemented
	// result = openConnectionToMyDevice();

	result = C_SUCCESS;

	D.connect();

	//updated devce status

	if (result)
	{
		m_deviceReady = true;
		return (C_SUCCESS);
	}
	else
	{
		m_deviceReady = false;
		return (C_ERROR);
	}

	newNeutralPos = false;
}
	

//----------------------------------------------------------------------------
/*!
Close connection to your device.

\return  __true__ if successful, __false__ otherwise.
*/
//----------------------------------------------------------------------------
bool c4dofChaiDevice::close()
{
	// check if the system has been opened previously
	if (!m_deviceReady) return (C_ERROR);

	bool result = C_SUCCESS; // if the operation fails, set value to C_ERROR.

								// *** INSERT YOUR CODE HERE ***
								// result = closeConnectionToMyDevice()

								// update status
	m_deviceReady = false;

	D.setForcesAndTorques(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	D.motorLoop();

	D.disconnect();

	return (result);
}


//----------------------------------------------------------------------------
// Calibrate your device.
//----------------------------------------------------------------------------
bool c4dofChaiDevice::calibrate(bool a_forceCalibration)
{
	bool result = C_SUCCESS;

	result = D.calibrate();

	return(result);
}


//----------------------------------------------------------------------------
// Returns the number of devices available from this class of device
//----------------------------------------------------------------------------
unsigned int c4dofChaiDevice::getNumDevices()
{
	//switch to 1 if you only want one finger/device
	int numberOfDevices = 2;

	return(numberOfDevices);
}


//----------------------------------------------------------------------------
/*
	Read the position of the device. Units are in [m].

	\param a_position Return location.

	\return __true__ if successful, __false__ otherwise
*/
//----------------------------------------------------------------------------
bool c4dofChaiDevice::getPosition(cVector3d& a_position)
{

	bool result = C_SUCCESS;

	// *** INSERT YOUR CODE HERE, MODIFY CODE BELLOW ACCORDINGLY ***
	// these axes align assuming the box is facing you and the chord of the tracker faces the box.
#ifdef MAGTRACKER
	// get position of this tracker
	pose = MT.CheckTrackerPose();
	pos = pose.getLocalPos();

	x = pos.x();
	y = pos.y();
	z = pos.z();
#endif

	// store new position values
	a_position.set(x, y, z);

	// transformation to go from mag tracker at back, to at finger tip (x red, y green, z blue)
	posOffsetinH.set(-0.07, 0, .09);


	//chai3d::cMatrix3d R_BtoA = pose.getLocalRot();
	//chai3d::cVector3d posOffsetinA;
	//R_BtoA.mulr(posOffsetinB, posOffsetinA);
	//a_position = a_position + posOffsetinA;

	// estimate linear velocity
	estimateLinearVelocity(a_position);

	// exit
	return (result);
}


//----------------------------------------------------------------------------
/*
Read the rotation of the device. Units are in [m].

\param a_rotation Return location.

\return __true__ if successful, __false__ otherwise
*/
//----------------------------------------------------------------------------
bool c4dofChaiDevice::getRotation(cMatrix3d& a_rotation)
{

	bool result = C_SUCCESS;

	// variables that describe the rotation matrix
	/*double r00, r01, r02, r10, r11, r12, r20, r21, r22;
	cMatrix3d frame; cMatrix3d deviceRotation;

	frame.identity();*/
	a_rotation.identity();

#ifdef MAGTRACKER
	// try setting rotation from mag tracker
	//frame = poseCache.getLocalRot();
	a_rotation = pose.getLocalRot();
#endif

	//determines how much tilt of the finger there is in the visualization
	double tilt = 15;

	a_rotation.rotateAboutLocalAxisDeg(0, 1, 0, 90 - tilt);

	//rotate to addjust for offset between end effector and magnetic traker location


	if (location == 0)
	{
		a_rotation.rotateAboutLocalAxisDeg(0, 1, 0, 9); //edit hereif change angle of finger        

	}
	if (location == 1)
	{
		a_rotation.rotateAboutLocalAxisDeg(0, 1, 0, 9 - tilt);
	}

	// store new rotation matrix
	/*a_rotation = frame;*/

	// estimate angular velocity
	estimateAngularVelocity(a_rotation);

	// exit
	return result;
}


//----------------------------------------------------------------------------
/*!
		Read the gripper angle in radian.

		\param   a_angle  Return value.

		\return  __true__ if successful, __false__ otherwise.
*/
//----------------------------------------------------------------------------
bool c4dofChaiDevice::getGripperAngleRad(double& a_angle)
{

	bool result = C_SUCCESS;


	// return gripper angle in radian
	a_angle = 0.0;  // a_angle = getGripperAngleInRadianFromMyDevice();

	// estimate gripper velocity
	estimateGripperVelocity(a_angle);

	// exit
	return (result);
}


//----------------------------------------------------------------------------
/*!
Send a force [N] and a torque [N*m] and gripper torque [N*m] to the haptic device.

\param   a_force  Force command.
\param   a_torque  Torque command.
\param   a_gripperForce  Gripper force command.

\return  __true__ if successful, __false__ otherwise.
*/
//----------------------------------------------------------------------------
bool c4dofChaiDevice::setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque,
	const double a_gripperForce)
{
	bool result = C_SUCCESS;

	//set the new neutral position if there is one
	if (newNeutralPos) {
		D.setNeutralPos(neutralPos[0], neutralPos[1], neutralPos[2], neutralPos[3]);
		newNeutralPos = false;
	}

	if (newTorsionState) {
		D.setTorsionState(torsionState);
		newTorsionState = false;
	}
	// calculate pad positions and gripper force
	D.setForcesAndTorques(a_force, a_torque);
	// Output to motors
	D.motorLoop();

	// exit
	return (result);
}



//==============================================================================
/*!
Read the status of the user switch [__true__ = \e ON / __false__ = \e OFF].

\param   a_switchIndex  index number of the switch.
\param   a_status result value from reading the selected input switch.

\return  __true__ if successful, __false__ otherwise.
*/
//==============================================================================
bool c4dofChaiDevice::getUserSwitch(int a_switchIndex, bool& a_status)
{

	bool result = C_SUCCESS;

	a_status = false;  // a_status = getUserSwitchOfMyDevice(a_switchIndex)

	return (result);
}

//! This method, instead of returnin the joint angles of the haptic device, returns the pantograph offset from the center position
bool c4dofChaiDevice::getJointAnglesRad(double a_jointAnglesRad[C_MAX_DOF]) {
	Eigen::Vector4d fingerPos;
	Eigen::Vector4d thumbPos;

	D.getc4DOFGripperPos(fingerPos);
	a_jointAnglesRad[0] = fingerPos[0];
	a_jointAnglesRad[1] = fingerPos[1];
	a_jointAnglesRad[2] = fingerPos[2];
	a_jointAnglesRad[3] = fingerPos[3];

	return C_SUCCESS;
}



bool c4dofChaiDevice::getNeutralPos(double &x, double &y, double &z, double &theta) {
	Eigen::Vector4d fingerNeutralPos(0, 0, 0, 0);
	D.getNeutralPos(fingerNeutralPos);

	x = fingerNeutralPos.x();
	y = fingerNeutralPos.y();
	z = fingerNeutralPos.z();
	theta = fingerNeutralPos.w();


	neutralPos[0] = fingerNeutralPos[0];
	neutralPos[1] = fingerNeutralPos[1];
	neutralPos[2] = fingerNeutralPos[2];
	neutralPos[3] = fingerNeutralPos[3];


	return C_SUCCESS;
	
}

bool c4dofChaiDevice::getDevicePos(double &x, double &y, double &z, double &theta) {
	Eigen::Vector4d devicePos(0, 0, 0, 0);
	D.getc4DOFGripperPos(devicePos);

	x = devicePos.x();
	y = devicePos.y();
	z = devicePos.z();
	theta = devicePos.w();
	return C_SUCCESS;
}

bool c4dofChaiDevice::theta_tracking(double& theta_des, double& theta_actual) {

	D.theta_tracking_device(theta_des, theta_actual);

	return C_SUCCESS;
}


bool c4dofChaiDevice::setNeutralPosX(const double &x) {
	double x_val = x;
	newNeutralPos = true;
	neutralPos[0] = x_val;

	return C_SUCCESS;
}

bool c4dofChaiDevice::setNeutralPosY(const double &y) {
	double y_val = y;
	newNeutralPos = true;
	neutralPos[1] = y_val;

	return C_SUCCESS;
}

bool c4dofChaiDevice::setNeutralPosZ(const double &z) {
	double z_val = z;
	newNeutralPos = true;
	neutralPos[2] = z_val;

	return C_SUCCESS;
}

bool c4dofChaiDevice::setNeutralPosTheta(const double &theta) {
	double theta_val = theta;
	newNeutralPos = true;
	neutralPos[3] = theta_val;

	return C_SUCCESS;
}

bool c4dofChaiDevice::setTorsionState(const bool &state) {
	newTorsionState = true;
	torsionState = state;

	return C_SUCCESS;
}


}

//------------------------------------------------------------------------------
#endif  // C_ENABLE_CUSTOM_DEVICE_SUPPORT
//------------------------------------------------------------------------------
