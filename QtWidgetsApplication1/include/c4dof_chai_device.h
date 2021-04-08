/*
\author <http://wwww.chai3d.org>
\author Sophia Williams, Stanford University
\version 3.2.0
*/
#pragma once


#include "devices/CGenericHapticDevice.h"
#include "magtracker.h"
#include "c4dofGripper.h"
#include <algorithm>




namespace chai3d {

	class c4dofChaiDevice;
	typedef std::shared_ptr<c4dofChaiDevice> c4dofChaiDevicePtr;

	/*
	\class	c4dofChaiDevice
	\ingroup	devices

	\brief
	Interface to custom c4dof haptic device based on the Chai3d template

	\details
	c4dofDevice is a 4dof parallel mechanism that can move in all 3 translation dof and 1 rotation dof.
	The device is a parallel mechanism. This code allows one to interface and control the haptic device.
	*/


	class c4dofChaiDevice : public cGenericHapticDevice
	{

		//---------------------------------------------------------------------------------
		//CONSTRUCTOR & DESTRUCTOR
		//---------------------------------------------------------------------------------

	public:

		//! Constructor of c4dofChaiDevice.
		c4dofChaiDevice(unsigned int a_deviceNumber);

		//! Destructor of c4dofChaiDevice.
		virtual ~c4dofChaiDevice();

		//! Shared c4dofChaiDevice allocator
		static c4dofChaiDevicePtr create(unsigned int a_deviceNumber) { return (std::make_shared<c4dofChaiDevice>(a_deviceNumber)); }


		//---------------------------------------------------------------------------------
		//Public Methods
		//---------------------------------------------------------------------------------

	public:

		//! Open connection to haptic device.
		virtual bool open();

		//! Close connection to haptic device.
		virtual bool close();

		//! Calibrate haptic device.
		virtual bool calibrate(bool a_forceCalibration = false);

		//! Read the position of the deivce
		virtual bool getPosition(cVector3d& a_position);

		//! Read the rotation of the deivce
		virtual bool getRotation(cMatrix3d& a_rotation);

		//! Read the gripper angle in radian [rad].
		virtual bool getGripperAngleRad(double& a_angle);

		//! Send a force [N] and a torque [N*m] and gripper force [N] to haptic deivce.
		virtual bool setForceAndTorqueAndGripperForce(const cVector3d& a_force, const cVector3d& a_torque,
			const double a_gripperForce);

		//! Read status of the switch [__true__ = __ON__/ __false__ = __OFF__].
		virtual bool getUserSwitch(int a_switchIndex, bool& a_status);

		//! This method returns the joint angles of the haptic device.
		virtual bool getJointAnglesRad(double a_jointAnglesRad[C_MAX_DOF]);

		//! This method is for Sophia's 4dof device and allows the user to read the Neutral Position of the device
		virtual bool getNeutralPos(double &x, double &y, double &z, double &theta);

		//! This is a method to grab the positions so that the user may access them for the GUI
		virtual bool getDevicePos(double &x, double &y, double &z, double &theta);

		//! This is a method to grab the actual and desired motor angle for the GUI
		virtual bool theta_tracking(double& theta_des, double& theta_actual);

		//! This method is for Sophia's 4dof device and allows the user to set the X Neutral Position
		virtual bool setNeutralPosX(const double &x);

		//! This method is for Sophia's 4dof device and allows the user to set the Y Neutral Position
		virtual bool setNeutralPosY(const double &y);

		//! This method is for Sophia's 4dof device and allows the user to set the Z Neutral Position
		virtual bool setNeutralPosZ(const double &z);

		//! This method is for Sophia's 4dof device and allows the user to set the Theta Neutral Position
		virtual bool setNeutralPosTheta(const double &theta);

		virtual bool setTorsionState(const bool &state);


		//--------------------------------------------------------------------------
		// PUBLIC STATIC METHODS:
		//--------------------------------------------------------------------------
	public:

		//! Returns the number of deivces available from this class of device.
		static unsigned int getNumDevices();

		//--------------------------------------------------------------------------
		// PUBLIC STATIC METHODS:
		//--------------------------------------------------------------------------
		bool hapticsOn = true;
		chai3d::cTransform pose;
		double scaleFactor;
		chai3d::cVector3d pos;
		chai3d::cVector3d centerPoint;
		chai3d::cVector3d scaledCenterPoint;
		chai3d::cVector3d scaledPos;

		chai3d::cVector3d posOffsetinH;

		magTrackerThread MT;		//position and orientation readings
		c4DOFGripper D;		//device angle readings and haptic output
		int location; //indicates the location or device number

	protected:

		double c4dofStartAngle = 0.0;
		double x = 0.0;
		double y = 0.0;
		double z = -15.0;


		/*instead of connecting the deivce to the chai device, c4dofChaiDevice passes
		forces and torques back to the haptics thread, and the haptics thread sets
		the c4dof forces and torques to the c4dof object*/

		bool newNeutralPos;
		vector<double> neutralPos;

		bool newTorsionState;
		bool torsionState;



	};



};
