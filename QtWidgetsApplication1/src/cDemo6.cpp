//===========================================================================
/*
Software License Agreement (BSD License)
Copyright (c) 2003-2016, CHAI3D
(www.chai3d.org)

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:

* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above
copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided
with the distribution.

* Neither the name of CHAI3D nor the names of its contributors may
be used to endorse or promote products derived from this software
without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

\author    <http://www.chai3d.org>
\author    Francois Conti
\version   3.2.0 $Rev: 1925 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CDemo6.h"
using namespace std;
// MPC Resource: https://github.com/michele-segata/mpclib

//---------------------------------------------------------------------------

//===========================================================================
/*!
Constructor of cDemo6.
*/
//===========================================================================
cDemo6::cDemo6(const string a_resourceRoot,
	const int a_numDevices,
	shared_ptr<cGenericHapticDevice> a_hapticDevice0,
	shared_ptr<cGenericHapticDevice> a_hapticDevice1, double maxStiffness) :cGenericDemo(a_resourceRoot, a_numDevices, a_hapticDevice0, a_hapticDevice1, maxStiffness)
{

	// store pointer to haptic device
	m_hapticDevice0 = a_hapticDevice0;

	matBase.setGrayLevel(0.3);
	matBase.setStiffness(0.0); // maxStiffness);

							   // define some material properties for each cube
	mat.setPurple();
	mat.m_specular.set(0.0, 0.0, 0.0);
	mat.setStiffness(maxStiffness);
	mat.setDynamicFriction(1.9);
	mat.setStaticFriction(1.9);

	target = new cShapeBox(targetWidth, targetLength, targetHeight);
	target->setLocalPos(0.0, 0.0, 0.0);
	m_ODEWorld->addChild(target);

	// initialize
	init();
};

cDemo6::~cDemo6() {
	//delete m_hapticDevice0;
	delete m_lqrController;
	delete target;
};

//===========================================================================
/*!
Set stiffness of environment

\param  a_stiffness  Stiffness value [N/m]
*/
//===========================================================================
void cDemo6::setStiffness(double a_stiffness)
{
	// set ground
	m_ground->setStiffness(a_stiffness, true);

};


//===========================================================================
/*!
Initialize position of objects
*/
//===========================================================================
void cDemo6::init()
{
	//guidanceOn = true;
	demo6Clock.start();
	lastCommandTime = 0.0;
	X1 = 0.0;
	Y1 = 0.0;
	X2 = 0.0;
	Y2 = 0.0;
}

void cDemo6::updateHaptics() {
	if (guidanceOn) {
		computeGuidanceForces();
	}
	cGenericDemo::updateHaptics();
}


void cDemo6::computeGuidanceForces()
{
	guidanceTimeStep = demo6Clock.getCurrentTimeSeconds()-guidanceTimeLast;
	guidanceTimeLast = demo6Clock.getCurrentTimeSeconds();
	
	bodyRotationMat = m_tool0->getDeviceGlobalRot();
	bodyRotationMat.trans();

	// find closest point in guidewire to center of fingers
	meanPos = (m_tool0->m_hapticPointFinger->getGlobalPosGoal() + m_tool0->m_hapticPointThumb->getGlobalPosGoal()) / 2.0;

	cVector3d indexToThumbVec = m_tool0->m_hapticPointThumb->getGlobalPosGoal() - m_tool0->m_hapticPointFinger->getGlobalPosGoal();
	cVector3d vertex;
	cMatrix3d targetRotMat;


	double t = demo6Clock.getCurrentTimeSeconds();	
	//vertex = cVector3d(0.04*cos(t) + .02, 0.0, 0.0); // 0.03*sin(t) + 0.04);
	//targetRotMat = cRotEulerDeg(20.0*sin(t) + 20.0, 0.0, 0.0, C_EULER_ORDER_ZYX);

	getTrajectory(vertex, targetRotMat, t);

	target->setLocalPos(vertex);
	target->setLocalRot(targetRotMat);

		// world frame translation error
		cVector3d translationError_world = vertex - meanPos;

		
		// body frame translation error
		translationError_body.set(0.0, 0.0, 0.0);
		bodyRotationMat.mulr(translationError_world, translationError_body);

		// rotation error
		//cMatrix3d bodyRotationMatTrans;
		//bodyRotationMat.transr(bodyRotationMatTrans);
		cMatrix3d errorRot;
		targetRotMat.mulr(bodyRotationMat, errorRot);
		//cMatrix3d errorRotTrans;
		//errorRot.transr(errorRotTrans);
		cVector3d eulerError = Rot2Eul(errorRot);
		
		// roll error (get angle in z-y plane of gripper frame)
		dRoll_error = (eulerError(2) - roll_error) / guidanceTimeStep;
		roll_error = eulerError(2);
		iRoll_error += roll_error*guidanceTimeStep;

		// yaw error (get angle in x-y plane of gripper frame)
		dYaw_error = (eulerError(0) - yaw_error) / guidanceTimeStep;
		yaw_error = eulerError(0);
		iYaw_error += yaw_error*guidanceTimeStep;


	// apply guidance forces
		if (m_naiveGuidance) {
			computeGuidanceForcesNaive();
		}
		else { // LQR guidance
			computeGuidanceForcesLQR();
		}


}

void cDemo6::computeGuidanceForcesLQR() {
	double time = demo6Clock.getCurrentTimeSeconds();
	double deltaT = time - lastCommandTime;
	lastCommandTime = time;

	// get the current desired pantograph position (relative to the center positions)
	double pantographPos[4];
	m_hapticDevice0->getJointAnglesRad(pantographPos); // overwritten to return the pantograph cartesian positions, not the joint angles
	X1 = pantographPos[0];
	Y1 = pantographPos[1];
	X2 = pantographPos[2];
	Y2 = pantographPos[3];

	// update state sent to LQR command generator
	m_lqrController->updateAll(meanPos, Rot2Eul(bodyRotationMat), -translationError_body(0), -translationError_body(2), -roll_error, -yaw_error); // pass negative error values to LQR

																																				  // get command for fingerpad velocities (index X, index Y, thumb X, thumb Y)
	Eigen::Matrix<float, 4, 1, Eigen::DontAlign> command = m_lqrController->getCommand();
	double mult = 1.0;

	if (deltaT < 0.01) {
		X1 = min(max(X1 + mult*command(0) * deltaT, -MAX_STRETCH), MAX_STRETCH); // TO DO: get current positions from actual pantograph code? send gripper object to lqr through generic demo code?
		Y1 = min(max(Y1 + mult*command(1) * deltaT, -MAX_STRETCH), MAX_STRETCH);
		X2 = min(max(X2 + mult*command(2) * deltaT, -MAX_STRETCH), MAX_STRETCH);
		Y2 = min(max(Y2 + mult*command(3) * deltaT, -MAX_STRETCH), MAX_STRETCH);

		fingerGuidance.set(-X1*k_skin, 0.0, -Y1*k_skin);	// flip x command on index because it will get flipped back in pantograph code
		thumbGuidance.set(X2*k_skin, 0.0, -Y2*k_skin);		// flip y commands on both because they will get flipped back in pantograph code
	}

	if (0) { // DEBUG
		cout << time << ", ";
		//cout << m_lqrController->getState().adjoint() << ", " << endl;
		//cout << translationError_body << ",    "; // << endl;
		//cout << yaw_error << ", " << roll_error << ", "; // << endl;
		cout << "Commanded Positions: "  << X1 << ", " << Y1 << " ," << X2 << ", " << Y2 << ", ";
		//cout << command(0) << ", " << command(1) << ", " << command(2) << ", " << command(3) << ", ";
		//cout << fingerGuidance << ", " << thumbGuidance; // << ", ";
		cout << endl;
	}
}

void cDemo6::computeGuidanceForcesNaive() {
	// add translation component of guidance
	thumbGuidance = K_translate*translationError_body + Kd_translate*dTranslationError_body;
	fingerGuidance = K_translate*translationError_body + Kd_translate*dTranslationError_body;

	// add rotation component of guidance
	thumbGuidance.add(-(K_rotate*yaw_error+Kd_rotate*dYaw_error), 0.0, -(K_rotate*roll_error+Kd_rotate*dRoll_error));
	fingerGuidance.add(K_rotate*yaw_error + Kd_rotate*dYaw_error, 0.0, K_rotate*roll_error + Kd_rotate*dRoll_error);

	if (0) { // DEBUG
		cout << translationError_body << ",    "; // << endl;
		cout << "Yaw error: " << yaw_error << "   Roll error: " << roll_error << ",   "; // << endl;
		//cout << fingerGuidance << ",    " << thumbGuidance;
		cout << endl;
	}
}

cVector3d cDemo6::Rot2Eul(cMatrix3d R) {

	float yaw = atan2(R(1, 0), R(0, 0));
	float pitch = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
	float roll = atan2(R(2, 1), R(2, 2));


	return cVector3d(yaw, pitch, roll);
}

void cDemo6::getTrajectory(cVector3d& vertex, cMatrix3d& targetRotMat, double t) {
	int trialNumAdjusted = trialNum;
	trialNumAdjusted = trialNum % 4;
	if (trialNumAdjusted == 0) trialNumAdjusted = 4;
	//if (trialNum > 4) {
	//	trialNumAdjusted -= 4;
	//}
	//cout << "Trajectory number: " << trialNumAdjusted << endl;
	/*switch (trialNumAdjusted) {
	case 1:
		vertex = cVector3d(0.04*cos(t) + .04 + .02, 0.0, 0.03*sin(t) + 0.05);
		targetRotMat = cRotEulerDeg(0.0, 0.0, 0.0, C_EULER_ORDER_ZYX);
		break;
	case 2:
		vertex = cVector3d(0.04, 0.0, 0.05);
		targetRotMat = cRotEulerDeg(20.0*sin(t) + 20.0, 0.0, 0.0, C_EULER_ORDER_ZYX);
		break;
	case 3:
		vertex = cVector3d(0.04+0.03*sin(t), 0.0, 0.05);
		targetRotMat = cRotEulerDeg(0.0, 0.0, 45.0, C_EULER_ORDER_ZYX);
		break;
	case 4:
		vertex = cVector3d(0.04, 0.02 + .04*sin(t/2), 0.05+0.04*sin(t/2));
		targetRotMat = cRotEulerRad(0.0, 0.0, -atan2(0.01,0.02), C_EULER_ORDER_ZYX);
		break;
	default:
		vertex.set(0.0, 0.0, 0.0);
		targetRotMat = cRotEulerDeg(0.0, 0.0, 0.0, C_EULER_ORDER_ZYX);
		break;
	}*/
	switch (trialNumAdjusted) {
	case 1:
		vertex = cVector3d(0.05, 0.0, 0.08);
		targetRotMat = cRotEulerDeg(0.0, 0.0, 0.0, C_EULER_ORDER_ZYX);
		break;
	case 2:
		vertex = cVector3d(0.01, 0.0, 0.02);
		targetRotMat = cRotEulerDeg(0.0, 0.0, 45.0, C_EULER_ORDER_ZYX);
		break;
	case 3:
		vertex = cVector3d(0.02, 0.02, 0.06);
		targetRotMat = cRotEulerDeg(0.0, 0.0, -45.0, C_EULER_ORDER_ZYX);
		break;
	case 4:
		vertex = cVector3d(0.0, -0.02, 0.05);
		targetRotMat = cRotEulerDeg(0.0, 0.0, 45.0, C_EULER_ORDER_ZYX);
		break;
	default:
		vertex.set(0.0, 0.0, 0.0);
		targetRotMat = cRotEulerDeg(0.0, 0.0, 0.0, C_EULER_ORDER_ZYX);
		break;
	}
	vertex.add(cVector3d(0.02, 0.0, 0.04));
}
//void cDemo6::applyOpenLoopInputs() {
//	translationError_body.set(0.0, 0.0, 0.0);
//	roll_error = 0.0;
//	yaw_error = 0.0;
//	double t = lastingClock.getCurrentTimeSeconds();
//
//	if (t < 12.0) {
//		translationError_body = cVector3d(0.0, 0.0, sinAmp*sin(((lastingClock.getCurrentTimeSeconds()) / 6)*lastingClock.getCurrentTimeSeconds()));
//	}
//	else if (t < 24.0) {
//		translationError_body = cVector3d(sinAmp*sin(((lastingClock.getCurrentTimeSeconds() - 12.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 12.0)), 0.0, 0.0);
//
//	}
//	else if (t < 36.0) {
//		translationError_body = cVector3d(sinAmp*sin(((lastingClock.getCurrentTimeSeconds() - 24.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 24.0)), 0.0, sinAmp*sin(((lastingClock.getCurrentTimeSeconds() - 24.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 24.0)));
//	}
//	else if (t < 48.0) {
//		translationError_body = cVector3d(-sinAmp*cos(((lastingClock.getCurrentTimeSeconds() - 36.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 36.0)), 0.0, sinAmp*cos(((lastingClock.getCurrentTimeSeconds() - 36.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 36.0)));
//	}
//	else if (t < 60.0) {
//		t -= 48.0;
//		yaw_error = 50 * sinAmp*sin((t / 6)*t);
//	}
//	else if (t < 70.0) {
//		t -= 60.0;
//		roll_error = 50 * sinAmp*sin((t / 6)*t);
//	}
//
//	else {
//		sinAmp = sinAmp / 2; // reduce magnitude and repeat
//		translationError_body.set(0.0, 0.0, 0.0);
//		yaw_error = 0.0;
//		roll_error = 0.0;
//		lastingClock.reset();
//	}
//
//}