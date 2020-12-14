#pragma once
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
\version   3.2.0 $Rev: 1869 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#ifndef CDemo6H
#define CDemo6H

//#define ringDemo
#define MAX_STRETCH 4.0			// maximum radius of stretch from centerpoint

//---------------------------------------------------------------------------
#include "chai3d.h"
#include "CGenericDemo.h"
#include "LQRController.h"
//---------------------------------------------------------------------------

class cDemo6 : public cGenericDemo
{
	//-----------------------------------------------------------------------
	// CONSTRUCTOR & DESTRUCTOR:
	//-----------------------------------------------------------------------

public:

	//! Constructor of cDemo6.
	cDemo6(const std::string a_resourceRoot,
		const int a_numDevices,
		std::shared_ptr<cGenericHapticDevice> a_hapticDevice0,
		std::shared_ptr<cGenericHapticDevice> a_hapticDevice,
		double maxStiffness);

	//! Destructor of cDemo5.
	virtual ~cDemo6();



	//-----------------------------------------------------------------------
	// PUBLIC METHODS:
	//-----------------------------------------------------------------------

public:

	//! Initialize demo
	virtual void init();

	//! Set stiffness
	virtual void setStiffness(double a_stiffness);

	void computeGuidanceForces();
	//void changeGuideWire(double w, double amplitude, double dh);

	cVector3d Rot2Eul(cMatrix3d R);
	void updateHaptics();

	//-----------------------------------------------------------------------
	// PUBLIC MEMBERS:
	//-----------------------------------------------------------------------

public:

	//! ODE objects
	cODEGenericBody* m_ODEBody0;
	cODEGenericBody* m_ODEBody1;
	cODEGenericBody* m_ODEBody2;

	cODEGenericBody* m_ODEBase0;
	cODEGenericBody* m_ODEBase1;
	cODEGenericBody* m_ODEBase2;
	cODEGenericBody* m_ODEBase3;
	cODEGenericBody* m_ODEBase4;
	cODEGenericBody* m_ODEBase5;

	// materials
	cMaterial matBase;
	cMaterial mat;

	// target object
	cShapeBox* target;
	double targetWidth = 0.03;
	double targetHeight = 0.015;
	double targetLength = 0.01;

	// error from target point
	cVector3d translationError_body;
	cVector3d dTranslationError_body;
	cVector3d iTranslationError_body;

	double roll_error, yaw_error;
	double dRoll_error, dYaw_error;
	double iRoll_error, iYaw_error;
	double guidanceTimeStep;
	double guidanceTimeLast;

	// skin stiffness for translating position commands to forces (match to pantograph.h )
	double k_skin = 1.58;	// [N/mm] scale force to skin displacement (~2 from provancher gleeson paper https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=5648412)

	// clock
	cPrecisionClock demo6Clock;
	int trialNum = 0;

private:

	shared_ptr<cGenericHapticDevice> m_hapticDevice0;

	// guidance
	double K_translate = 200.0 ; //160.0; // TO DO: tune this
	double K_rotate = 20.0; //8.0;
	double Kd_translate = 0.02;
	double Kd_rotate = 0.02;
	double Ki_translate = 0.0;
	double Ki_rotate = 0.0;

	// position information
	cVector3d meanPos;
	cMatrix3d bodyRotationMat;

	// open loop guidance data collection 
	double sinAmp = 0.03;
	//void applyOpenLoopInputs();
	//bool openLoopInputs = true;
	
	void getTrajectory(cVector3d& vector, cMatrix3d& targetRotMat, double t);

	// naive guidance
	void computeGuidanceForcesNaive();

	// lqr guidance
	void computeGuidanceForcesLQR();
	LQRController* m_lqrController = new LQRController;
	double lastCommandTime;
	double X1, Y1, X2, Y2;
};

//---------------------------------------------------------------------------
#endif
//---------------------------------------------------------------------------
