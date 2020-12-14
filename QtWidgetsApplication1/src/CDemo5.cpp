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
#include "CDemo5.h"
using namespace std;
extern "C" {
#include "solver.h"
}
//---------------------------------------------------------------------------


//===========================================================================
/*!
    Constructor of cDemo5.
*/
//===========================================================================
cDemo5::cDemo5(const string a_resourceRoot,
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
	
	// original ring object
#ifdef ringDemo
	m_ODEBase0 = new cODEGenericBody(m_ODEWorld);
	cMesh* base0 = new cMesh();
	cCreateCylinder(base0, 0.15, 0.01, 12, 1, 1, true, true, cVector3d(0, 0, -0.075), cIdentity3d(), cColorf(1.0, 1.0, 1.0));
	m_ODEBase0->createDynamicCylinder(0.01, 0.15, true);
	base0->createAABBCollisionDetector(m_toolRadius);
	base0->setMaterial(matBase);
	m_ODEBase0->setImageModel(base0);
	m_ODEBase0->setLocalPos(0.0, 0.0, -0.02); //0.015, 0.0, -0.02);
#endif



	ring0 = new cMesh();
	cCreateRing(ring0, innerRad, outerRad, 12, 16);
	ring0->setMaterial(mat);
	ring0->rotateAboutLocalAxisDeg(cVector3d(1.0, 0.0, 0.0), 90.0);
	ring0->setHapticEnabled(false); 
	m_tool0->m_hapticPointFinger->m_sphereProxy->addChild(ring0);
	ring0->setLocalPos(-outerRad*.9, -(innerRad+0.005), 0.0);

	m_guideWire = new cMultiSegment();
	buildGuideWire(50, .03, 0.0);
	m_ODEWorld->addChild(m_guideWire);

 if (0){
		// add sine curve to world
		double w = 50;
		double h = 0.0;
		double dh = 0.0;
		double a = 0.0;
		double da = 0.001;
		double r = 0.03;
		double x0 = 0.025;
		double y0 = -0.1250;
		double z0 = 0.08;//-.04;

		for (int i = 0; i < 250; i++)
		{
			double py0 = a;
			double pz0 = r*sin(w*a);
			double px0 = h;
			double py1 = a+da;
			double pz1 = r*sin(w*(a + da));
			double px1 = h+dh;
			// create vertex 0
			int index0 = m_guideWire->newVertex(px0 + x0, py0 + y0, pz0 + z0);

			// create vertex 1
			int index1 = m_guideWire->newVertex(px1 + x0, py1 + y0, pz1 + z0);

			// create segment by connecting both vertices together
			m_guideWire->newSegment(index0, index1);
			h = h + dh;
			a = a + da;
		}
		// set haptic properties
		m_guideWire->setMaterial(mat);
		m_guideWire->m_material->setStiffness(0);

		// assign color properties
		cColorf color;
		color.setYellowGold();
		m_guideWire->setLineColor(color);
		// assign line width
		m_guideWire->setLineWidth(10.0);
		// use display list for faster rendering
		m_guideWire->setUseDisplayList(true);
		// build collision tree
		m_guideWire->createAABBCollisionDetector(m_toolRadius);
		m_guideWire->setHapticEnabled(false);
		m_ODEWorld->addChild(m_guideWire);	// connect some segments to form a spring

	}

    // initialize
   // init();
};

cDemo5::~cDemo5() {
	//delete m_hapticDevice0;
	delete m_lqrController;
	if (NULL != m_guideWire) {
		delete m_guideWire;
	}
}; 

//===========================================================================
/*!
    Set stiffness of environment

    \param  a_stiffness  Stiffness value [N/m]
*/
//===========================================================================
void cDemo5::setStiffness(double a_stiffness)
{
    // set ground
    m_ground->setStiffness(a_stiffness, true);
    
#ifdef ringDemo
    // set objects
    m_ODEBody0->m_imageModel->setStiffness(a_stiffness);
    m_ODEBase0->m_imageModel->setStiffness(a_stiffness);
#endif
};


//===========================================================================
/*!
    Initialize position of objects
*/
//===========================================================================
void cDemo5::init()
{
	char input;
	cout << endl << endl << "Initialization Sequence? Y/N: ";
	//cin >> input;
	input = 'n';
	if (input == 'Y' || input == 'y') {
		openLoopInputs = true;
		cout << "Starting initialization sequence. " << endl;
	}
	else {
		openLoopInputs = false;
		cout << " Proceeding with regular trials." << endl;
	}

	guidanceOn = true;
	lastingClock.start();
	lastCommandTime = 0.0;
	X1 = 0.0;
	Y1 = 0.0;
	X2 = 0.0;
	Y2 = 0.0;

	//m_ground->setEnabled(false);
	m_ground->setStiffness(0.0);
	//m_ground->setShowEnabled(false);
	m_ground->setHapticEnabled(false);
}

void cDemo5::updateHaptics() {

	if (guidanceOn) {
		computeGuidanceForces();
	}

	cGenericDemo::updateHaptics();

}



void cDemo5::computeGuidanceForces() 
{

	bodyRotationMat = m_tool0->getDeviceGlobalRot();
	bodyRotationMat.trans();

	// find closest point in guidewire to center of fingers
	meanPos = (m_tool0->m_hapticPointFinger->getGlobalPosGoal() + m_tool0->m_hapticPointThumb->getGlobalPosGoal()) / 2.0;
	cVector3d ringOffset;
	bodyRotationMat.mulr(cVector3d(-outerRad, 0.0, 0.0), ringOffset);
	meanPos.add(ringOffset);

	cVector3d indexToThumbVec = m_tool0->m_hapticPointThumb->getGlobalPosGoal() - m_tool0->m_hapticPointFinger->getGlobalPosGoal();
	
	//guideWireUpdateMutex.acquire();

	// only update if wire is ready
	if (NULL != m_guideWire && m_guideWire->getNumVertices() > 0) {
		int vertexInd = closestWirePoint(meanPos);
		cVector3d vertex = m_guideWire->m_vertices->getLocalPos(vertexInd);

		// world frame translation error
		cVector3d translationError_world = vertex - meanPos;

		// body frame translation error
		translationError_body.set(0.0, 0.0, 0.0);
		bodyRotationMat.mulr(translationError_world, translationError_body);

		//cout << translationError_body.x() << "   " << translationError_body.z() << endl;

		// get desired rotation matrix
		cVector3d nextVertex = m_guideWire->m_vertices->getLocalPos(vertexInd + 1);
		cVector3d tangent_world = nextVertex - vertex;
		cVector3d tangent;
		bodyRotationMat.mulr(tangent_world, tangent);

		// roll error (get angle in z-y plane of gripper frame)
		roll_error = atan2(tangent.z(), tangent.y());
		// yaw error (get angle in x-y plane of gripper frame)
		yaw_error = atan2(tangent.x(), tangent.y());

	}


	//openLoopInputs = false;
	if (openLoopInputs) { 
		applyOpenLoopInputs(); 
		computeGuidanceForcesNaive();

	}
	// apply guidance forces if within a threshold of wire
	else if (translationError_body.length() < 0.07) {
		if (m_naiveGuidance) {
			computeGuidanceForcesNaive();
		}
		else { 
			// LQR  or MPC guidance
			computeOptimalGuidanceForces();
		}

	}

	// too far from wire
	else {
		thumbGuidance.set(0.0, 0.0, 0.0);
		fingerGuidance.set(0.0, 0.0, 0.0);
		X1 = 0.0; 
		Y1 = 0.0; 
		X2 = 0.0; 
		Y2 = 0.0;
	}
	//guideWireUpdateMutex.release();

}

void cDemo5::computeOptimalGuidanceForces() {
	double time = lastingClock.getCurrentTimeSeconds();
	double deltaT = time - lastCommandTime;
	if (deltaT >= 1.0/80.0) { // don't calculate more often than magtracker is updating 1.0/80.0
		lastCommandTime = time;

		// get the current desired pantograph position (relative to the center positions)
		double pantographPos[4];
		m_hapticDevice0->getJointAnglesRad(pantographPos); // overwritten to return the pantograph cartesian positions, not the joint angles
		X1 = pantographPos[0];
		Y1 = pantographPos[1];
		X2 = pantographPos[2];
		Y2 = pantographPos[3];


		// get command for fingerpad velocities (index X, index Y, thumb X, thumb Y)
		Eigen::Matrix<float, 4, 1, Eigen::DontAlign> command;

		if (m_mpcGuidance) {
			// update state sent to MPC command generator and get command
			m_mpcController->updateAll(meanPos, Rot2Eul(bodyRotationMat), -translationError_body(0), -translationError_body(2), -roll_error, -yaw_error, X1, Y1, X2, Y2); // pass negative error values to LQR
			command = m_mpcController->getMPCinputs();
		}
		else {
			// update state sent to LQR command generator and get command
			m_lqrController->updateAll(meanPos, Rot2Eul(bodyRotationMat), -translationError_body(0), -translationError_body(2), -roll_error, - yaw_error); // pass negative error values to LQR
			command = m_lqrController->getCommand();
		}

		double mult = 1.0;
		//if (deltaT < 0.25) {
			X1 = min(max(X1 + mult*command(0) * deltaT, -MAX_STRETCH), MAX_STRETCH); // TO DO: get current positions from actual pantograph code? send gripper object to lqr through generic demo code?
			Y1 = min(max(Y1 + mult*command(1) * deltaT, -MAX_STRETCH), MAX_STRETCH);
			X2 = min(max(X2 + mult*command(2) * deltaT, -MAX_STRETCH), MAX_STRETCH);
			Y2 = min(max(Y2 + mult*command(3) * deltaT, -MAX_STRETCH), MAX_STRETCH);

			fingerGuidance.set(-X1*k_skin, 0.0, -Y1*k_skin);	// flip x command on index because it will get flipped back in pantograph code
			thumbGuidance.set(X2*k_skin, 0.0, -Y2*k_skin);		// flip y commands on both because they will get flipped back in pantograph code


			if (1) { // DEBUG
				cout << time << ", ";
				//cout << m_lqrController->getState().adjoint() << ", " << endl;
				//cout << translationError_body << ",    "; // << endl;
				//cout  << yaw_error << ", " << roll_error <<", "; // << endl;
				cout << X1 << ", " << Y1 << " ," << X2 << ", " << Y2 << ", ";
				//cout << command(0)*deltaT << ", " << command(1)*deltaT << ", " << command(2)*deltaT << ", " << command(3)*deltaT << ", ";
				//cout << fingerGuidance << ", " << thumbGuidance; // << ", ";
				cout << endl;
			}
	//	}
	}
}

void cDemo5::computeGuidanceForcesNaive() {
	// add translation component of guidance
	thumbGuidance = K_translate*translationError_body;
	fingerGuidance = K_translate*translationError_body;

	// add rotation component of guidance
	thumbGuidance.add(-K_rotate*yaw_error, 0.0, -K_rotate*roll_error);
	fingerGuidance.add(K_rotate*yaw_error, 0.0, K_rotate*roll_error);

	if (0) { // DEBUG
		cout << translationError_body << ",    "; // << endl;
		cout << "Yaw error: " << yaw_error << "   Roll error: " << roll_error; // << endl;
		 cout << fingerGuidance << ", " << thumbGuidance;
	}
}

void cDemo5::changeGuideWire(double w, double amplitude, double dh) {
	guidanceOn = false;
	//double w = 50;
	double h = -100.0 * dh; // 0.0;
							//double dh = 0.0;
	double a = 0.0;
	double da = 0.001;
	//double amplitude = 0.03;
	double x0 = 0.025;
	double y0 = -0.1250;
	double z0 = 0.08;//-.04;

	cout << "Creating new Wire shape" << endl;
	for (int i = 0; i < 250; i++)
	{
		double py0 = a;
		double pz0 = amplitude*sin(w*a);
		double px0 = h;
		double py1 = a + da;
		double pz1 = amplitude*sin(w*(a + da));
		double px1 = h + dh;
		// create vertex 0
		//int index0 = m_guideWire->newVertex(px0 + x0, py0 + y0, pz0 + z0);
		m_guideWire->m_vertices->setLocalPos(2 * i, cVector3d(px0 + x0, py0 + y0, pz0 + z0));
		// create vertex 1
		m_guideWire->m_vertices->setLocalPos(2*i+1, cVector3d(px1 + x0, py1 + y0, pz1 + z0));

		m_guideWire->m_segments->setVertices(i, 2 * i, 2 * i + 1);
		m_guideWire->markForUpdate(true);
		// create segment by connecting both vertices together
		//m_guideWire->newSegment(index0, index1);
		h = h + dh;
		a = a + da;
	}
	guidanceOn = true;
	m_guideWire->setEnabled(true);

}

// update wire vertices 
void cDemo5::buildGuideWire(double w, double amplitude , double dh ) {
	guideWireUpdateMutex.acquire();
	guidanceOn = false;
	m_guideWire->removeFromGraph();
	m_guideWire->clear();

	//double w = 50;
	double h = -100.0 * dh; // 0.0;
	//double dh = 0.0;
	double a = 0.0;
	double da = 0.001;
	//double amplitude = 0.03;
	double x0 = 0.025;
	double y0 = -0.1250;
	double z0 = 0.08;//-.04;

	for (int i = 0; i < 250; i++)
	{
		double py0 = a;
		double pz0 = amplitude*sin(w*a);
		double px0 = h;
		double py1 = a + da;
		double pz1 = amplitude*sin(w*(a + da));
		double px1 = h + dh;
		// create vertex 0
		int index0 = m_guideWire->newVertex(px0 + x0, py0 + y0, pz0 + z0);
		// create vertex 1
		int index1 = m_guideWire->newVertex(px1 + x0, py1 + y0, pz1 + z0);

		// create segment by connecting both vertices together
		m_guideWire->newSegment(index0, index1);
		h = h + dh;
		a = a + da;
	}
	// set haptic properties
	m_guideWire->setMaterial(mat);
	m_guideWire->m_material->setStiffness(0);

	// assign color properties
	cColorf color;
	color.setYellowGold();
	m_guideWire->setLineColor(color);
	m_guideWire->setLineWidth(20.0);							// assign line width
	m_guideWire->setUseDisplayList(true);						// use display list for faster rendering
	m_guideWire->createAABBCollisionDetector(m_toolRadius); 	// build collision tree

	m_guideWire->setEnabled(true);
	m_guideWire->setHapticEnabled(false);
	m_ODEWorld->addChild(m_guideWire);
	guidanceOn = true;

	guideWireUpdateMutex.release();

}


int cDemo5::closestWirePoint(cVector3d pos) {
	int vertex = 0;
	double smallestDist = 1000.0;
	double a_dist = 0.0;
	
	int n = m_guideWire->getNumVertices();
	
	for (int ii = 0; ii < n; ii = ii + 2) {
		if (m_guideWire->getNumVertices() > 0) {
			a_dist = pos.distance(m_guideWire->m_vertices->getLocalPos(ii));
			// is this the best so far?
			if (a_dist < smallestDist) {
				vertex = ii;
				smallestDist = a_dist;
			}
		}
	}
	return vertex;
}


cVector3d cDemo5::Rot2Eul(cMatrix3d R) {

	float yaw = atan2(R(1, 0), R(0, 0));
	float pitch = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
	float roll = atan2(R(2, 1), R(2, 2));


	return cVector3d(yaw, pitch, roll);
}

void cDemo5::applyOpenLoopInputs() {
	translationError_body.set(0.0, 0.0, 0.0);
	roll_error = 0.0;
	yaw_error = 0.0;
	double t = lastingClock.getCurrentTimeSeconds();

	if (t < 12.0) {
		translationError_body = cVector3d(0.0, 0.0, sinAmp*sin(((lastingClock.getCurrentTimeSeconds()) / 6)*lastingClock.getCurrentTimeSeconds()));
	}
	else if (t < 24.0) {
		translationError_body = cVector3d(sinAmp*sin(((lastingClock.getCurrentTimeSeconds() - 12.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 12.0)), 0.0, 0.0);

	}
	else if (t < 36.0) {
		translationError_body = cVector3d(sinAmp*sin(((lastingClock.getCurrentTimeSeconds() - 24.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 24.0)), 0.0, sinAmp*sin(((lastingClock.getCurrentTimeSeconds() - 24.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 24.0)));
	}
	else if (t < 48.0) {
		translationError_body = cVector3d(-sinAmp*cos(((lastingClock.getCurrentTimeSeconds() - 36.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 36.0)), 0.0, sinAmp*cos(((lastingClock.getCurrentTimeSeconds() - 36.0) / 6)*(lastingClock.getCurrentTimeSeconds() - 36.0)));
	}
	else if (t < 60.0) {
		t -= 48.0;
		yaw_error = 50*sinAmp*sin((t/ 6)*t);
	}
	else if (t < 70.0) {
		t -= 60.0;
		roll_error = 50*sinAmp*sin((t / 6)*t);
	}

	else {
		sinAmp = sinAmp / 2; // reduce magnitude and repeat
		translationError_body.set(0.0, 0.0, 0.0);
		yaw_error = 0.0;
		roll_error = 0.0;
		lastingClock.reset();
	}

}