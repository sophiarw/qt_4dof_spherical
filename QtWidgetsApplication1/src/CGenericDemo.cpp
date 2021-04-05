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
    \version   3.2.0 $Rev: 1928 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "CGenericDemo.h"
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
#include <time.h>
//---------------------------------------------------------------------------
#include <QDebug>
//------------------------------------------------------------------------------
// DECLARED MACROS
//------------------------------------------------------------------------------
// convert to resource path
#define RESOURCE_PATH(p)    (char*)((a_resourceRoot+string(p)).c_str())



//===========================================================================
/*!
    Constructor of cGenericDemo.
*/
//===========================================================================
cGenericDemo::cGenericDemo(const string a_resourceRoot,
                           const int a_numDevices,
                           shared_ptr<cGenericHapticDevice> a_hapticDevice0,
                           shared_ptr<cGenericHapticDevice> a_hapticDevice1,
							double maxStiffness)
{
	setStiffness(maxStiffness);

	//initialize r vectors for torque calculations
	r = { 0.0, 0.0, 0.0 };
	r0 = { 0.0, 0.0, 0.0 };
	r1 = { 0.0, 0.0, 0.0 };


	// clamp the force output gain to the max device stiffness
	linGain = cMin(linGain, maxStiffness / linStiffness);

	// display is not mirrored
	m_mirroredDisplay = false;

	// torque gain
	m_torqueGain = 2.0;

	// initialize tool radius
	m_toolRadius = 0.008;

	//file name for debugging
	srand(time(NULL));
	filename = "example" + std::to_string(rand()) + ".csv";

	qDebug() << "The number of devices is" << a_numDevices << endl;

	// clamp the force output gain to the max device stiffness
	linGain = cMin(linGain, maxStiffness / linStiffness);

    // display is not mirrored
    m_mirroredDisplay = false;

	m_world = new cWorld();

	// set the background color of the environment
	m_world->m_backgroundColor.setBlack();

	// create a camera and insert it into the virtual world
	m_camera = new cCamera(m_world);
	m_world->addChild(m_camera);


	// define a basis in spherical coordinates for the camera
	m_camera->setSphericalReferences(cVector3d(0, 0, 0),    // origin
		cVector3d(0, 0, 1),    // zenith direction
		cVector3d(1, 0, 0));   // azimuth direction

	m_camera->setSphericalDeg(4.0,    // spherical coordinate radius
		45,      // spherical coordinate azimuth angle
		0);     // spherical coordinate polar angle


				// set the near and far clipping planes of the camera
	m_camera->setClippingPlanes(0.01, 20.0);

	// create a light source
	m_light = new cDirectionalLight(m_world);

	// add light to camera
	m_camera->addChild(m_light);

	// enable light source
	m_light->setEnabled(true);

	// define the direction of the light beam
	m_light->setDir(-1.0, -1.0, -0.5);
	
	//// create a label to display the haptic and graphic rate of the simulation
	cFontPtr font = NEW_CFONTCALIBRI32();
	labelTorq = new cLabel(font);
	labelTorq->m_fontColor.setBlack();
	m_camera->m_frontLayer->addChild(labelTorq);



    //// create a background
    cBackground* background = new cBackground();
    m_camera->m_backLayer->addChild(background);

	// label = 

    // set background properties
    background->setCornerColors(cColorf(1.00, 1.00, 1.00),
                                cColorf(1.00, 1.00, 1.0),
                                cColorf(0.90, 0.90, 0.90),
                                cColorf(0.90, 0.90, 0.90));

    // create a ground
    m_ground = new cMesh();
    m_world->addChild(m_ground);
    
    cCreatePlane(m_ground, 0.2, 0.3);
    m_ground->m_material->setGrayLevel(0.5);
    m_ground->m_material->setStiffness(maxStiffness);
    m_ground->m_material->setDynamicFriction(1.0);//original value 2.0, tried 1.0
    m_ground->m_material->setStaticFriction(4.0);//original value 2.0, tried 4.0
    m_ground->createAABBCollisionDetector(m_toolRadius);
    m_ground->setShowEnabled(false);

    // create a base
    m_base = new cMultiMesh();
    m_world->addChild(m_base);
    
    bool fileload = m_base->loadFromFile(RESOURCE_PATH("../../external/chai3d-3.2.0/modules/ODE/bin/resources/models/base/base.obj"));
    if (!fileload)
    {
        //fileload = m_base->loadFromFile("../../../bin/resources/models/base/base.obj");
		fileload = m_base->loadFromFile(RESOURCE_PATH("../../../external/chai3d-3.2.0/modules/ODE/bin/resources/models/base/base.obj"));

    }
    if (!fileload)
    {
        qDebug() << "Error - 3D Model failed to load correctly.\n";
    }

    m_base->scale(0.001);
    m_base->setShowFrame(false);
    m_base->setShowBoundaryBox(false);
    m_base->setLocalPos(-0.05, 0.0, 0.001);
    m_base->setUseDisplayList(true);

    // create tools
    m_tools[0] = NULL;
    m_tools[1] = NULL;
    m_tool0 = NULL;
    m_tool1 = NULL;

    m_numTools = cMin(a_numDevices, 2);

    cMesh* mesh = new cMesh();
    cMatrix3d rot;
    rot.identity();
    rot.rotateAboutLocalAxisDeg(cVector3d(1,0,0), 90);
    cCreateRing(mesh, 0.001, 0.005, 4, 16, cVector3d(0,0,0), rot);
    mesh->m_material->setWhite();
    mesh->setTransparencyLevel(0.4f);

	// FINGER MESHES
	chai3d::cMultiMesh* finger = new chai3d::cMultiMesh(); // create the finger
	finger->setFrameSize(0.01);
	finger->setLocalPos(0.0, 0.0, 0.0);

	chai3d::cMultiMesh* thumb = new chai3d::cMultiMesh(); //create the thumb
	thumb->setFrameSize(0.01);
	thumb->setLocalPos(0.0, 0.0, 0.0);


	if (cLoadFileOBJ(finger, "../../external/Resources/FingerModel.obj")) {
		qDebug() << "Finger file loaded. " << endl;
	}
	else if(cLoadFileOBJ(finger, "../../../external/Resources/FingerModel.obj")) {
		qDebug() << "Finger file loaded. " << endl;
	}
	else {
		qDebug() << "Failed to load finger model." << endl;
	}

	if (cLoadFileOBJ(thumb, "../../external/Resources/FingerModelThumb.obj")) {
		qDebug() << "Thumb file loaded. " << endl;
	}
	else if (cLoadFileOBJ(finger, "../../../external/Resources/FingerModelThumb.obj")) {
		qDebug() << "Finger file loaded. " << endl;
	}
	else {
		qDebug() << "Failed to load thumb model." << endl;
	}




	// set params for finger
	finger->scale(0.8);
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), 180);
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 90);
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 180);
	finger->setShowEnabled(true);
	finger->setUseVertexColors(true);
	chai3d::cColorf fingerColor;
	fingerColor.setBrownSandy();
	finger->setVertexColor(fingerColor);
	finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
	finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	finger->m_material->m_specular.set(1.0, 1.0, 1.0);
	finger->setUseMaterial(true);
	//finger->setHapticEnabled(false);
	finger->setShowFrame(false);
	finger->setStiffness(1000);

	// set params for thumb
	thumb->scale(0.8);
	thumb->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 90);
	thumb->setShowEnabled(true);
	thumb->setUseVertexColors(true);
	chai3d::cColorf thumbColor;
	thumbColor.setBrownSandy();
	thumb->setVertexColor(thumbColor);
	thumb->m_material->m_ambient.set(0.1, 0.1, 0.1);
	thumb->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	thumb->m_material->m_specular.set(1.0, 1.0, 1.0);
	thumb->setUseMaterial(true);
	//thumb->setHapticEnabled(false);
	thumb->setShowFrame(false);
	thumb->setStiffness(1000);


	// create lines to illustrate the force on the haptic device
	forceThumbLine = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	forceThumbLine->setLineWidth(4.0);
	forceThumbLine->m_material->setRedCrimson();
	forceFingerLine = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	forceFingerLine->setLineWidth(4.0);
	forceFingerLine->m_material->setGreenLawn();
	m_world->addChild(forceThumbLine);
	m_world->addChild(forceFingerLine);


	//added additional lines to debug
	torque_line = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	r_line = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	contact_line = new cShapeLine(cVector3d(0, 0, 0), cVector3d(0, 0, 0));
	
	//add to world
	m_world->addChild(r_line);
	m_world->addChild(torque_line);
	m_world->addChild(contact_line);

	torque_line->setLineWidth(4.0);
	torque_line->m_colorPointA.setPurpleDarkViolet();
	torque_line->m_colorPointB.setPurpleDarkViolet();
	r_line->setLineWidth(4.0);
	r_line->m_colorPointA.setRedCrimson();
	r_line->m_colorPointB.setRedCrimson();
	contact_line->setLineWidth(4.0);
	contact_line->m_colorPointA.setYellowLightGoldenrod();
	contact_line->m_colorPointB.setYellowLightGoldenrod();






    if (m_numTools > 0)
    {

        m_tool0 = new cToolCursor(m_world);
        m_world->addChild(m_tool0);
		m_tool0->setHapticDevice(a_hapticDevice0);
        m_tool0->setRadius(m_toolRadius);
        m_tool0->enableDynamicObjects(true);
        m_tool0->setWaitForSmallForce(true);
		m_tool0->setShowFrame(false,true);
		m_tool0->setFrameSize(0.05, true);
        m_tool0->start();


		
        //cMesh* mesh0 = mesh->copy();
      

       // cMesh* mesh1 = mesh->copy();
		m_tool0->getHapticPoint(0)->m_sphereProxy->addChild(finger);
		m_tool0->getHapticPoint(0)->m_sphereProxy->setTransparencyLevel(0.0);
		m_tool0->getHapticPoint(0)->m_sphereProxy->setShowFrame(false, true);
		m_tool0->getHapticPoint(0)->m_sphereProxy->setFrameSize(0.05);

		m_tools[0] = m_tool0;

    }

    if (m_numTools > 1)
    {
        m_tool1 = new cToolCursor(m_world);
        m_world->addChild(m_tool1);
        m_tool1->setHapticDevice(a_hapticDevice1);
        m_tool1->setRadius(m_toolRadius);
        m_tool1->enableDynamicObjects(true);
        m_tool1->setWaitForSmallForce(true);
		m_tool1->setShowFrame(false, true);
		m_tool1->setFrameSize(0.05, true);
        
        m_tools[1] = m_tool1;

		cHapticPoint* point1 = m_tool1->getHapticPoint(0);

		//Julie has this instead of the next for lines in this if statement
		/*m_tool1->m_hapticPointThumb->m_sphereProxy->addChild(thumb);
		cMesh* mesh1 = mesh->copy();
		m_tool1->m_hapticPointFinger->m_sphereProxy->addChild(finger);*/

		m_tool1->getHapticPoint(0)->m_sphereProxy->addChild(thumb);
		m_tool1->getHapticPoint(0)->m_sphereProxy->setTransparencyLevel(0.0);
		m_tool1->getHapticPoint(0)->m_sphereProxy->setShowFrame(false, true);
		m_tool1->getHapticPoint(0)->m_sphereProxy->setFrameSize(0.05);

		m_tool1->start();

    }

    // create an ODE world to simulate dynamic bodies
    m_ODEWorld = new cODEWorld(m_world);
    m_world->addChild(m_ODEWorld);

    // set some gravity
    m_ODEWorld->setGravity(cVector3d(0.0, 0.0, -9.81));
    m_ODEWorld->setLinearDamping(0.01);
    m_ODEWorld->setAngularDamping(0.01);

    // we create 6 static walls to contains the 3 cubes within a limited workspace
    m_ODEGPlane0 = new cODEGenericBody(m_ODEWorld);
    m_ODEGPlane1 = new cODEGenericBody(m_ODEWorld);
    m_ODEGPlane2 = new cODEGenericBody(m_ODEWorld);
    m_ODEGPlane3 = new cODEGenericBody(m_ODEWorld);
    m_ODEGPlane4 = new cODEGenericBody(m_ODEWorld);
    m_ODEGPlane5 = new cODEGenericBody(m_ODEWorld);

    double d = 0.14 / 2.0;
    double w = 0.20 / 2.0;
    double h = 0.60 / 2.0;

    m_ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, h), cVector3d(0.0, 0.0 ,-1.0));
    m_ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, 0.0), cVector3d(0.0, 0.0 , 1.0));
    m_ODEGPlane2->createStaticPlane(cVector3d(0.0,  w, 0.0), cVector3d(0.0,-1.0, 0.0));
    m_ODEGPlane3->createStaticPlane(cVector3d(0.0, -w, 0.0), cVector3d(0.0, 1.0, 0.0));
    m_ODEGPlane4->createStaticPlane(cVector3d( d, 0.0, 0.0), cVector3d(-1.0,0.0, 0.0));
    m_ODEGPlane5->createStaticPlane(cVector3d(-d, 0.0, 0.0), cVector3d( 1.0,0.0, 0.0));


}


//===========================================================================
/*!
    Update graphics.

    \param  a_width  Width of viewport.
    \param  a_height  Height of viewport.
*/
//===========================================================================
void cGenericDemo::updateGraphics(int a_width, int a_height) 
{ 


	guideWireUpdateMutex.acquire();

	// update haptic and graphic rate data
	labelTorq->setText("Angle: " + cStr(angle,5)+ ", Contact: " + cStr(prev_contact) + "\n Torque : " + cStr(m_torque.x()) + ", " + cStr(m_torque.y()) + ", " + cStr(m_torque.z()));

	// update position of label
	labelTorq->setLocalPos((int)100, 15);

	// render view
	m_camera->renderView(a_width, a_height);

    // update shadow maps (if any)
    m_world->updateShadowMaps(false, m_mirroredDisplay);
    
	//visualize lines for debugging
	//forceThumbLine->m_pointA = m_tool0->getHapticPoint(0)->getGlobalPosProxy();
	//forceThumbLine->m_pointB = forceThumbLine->m_pointA + m_force*0.005;
	
	torque_line->m_pointA = m_tools[0]->getHapticPoint(0)->getGlobalPosProxy();
	torque_line->m_pointB = torque_line->m_pointA + m_torque*0.01;
	/*
	contact_line->m_pointA = m_tool0->getHapticPoint(0)->getGlobalPosProxy();
	contact_line->m_pointB = forceThumbLine->m_pointA + finger_prime;

	r_line->m_pointA = object_pos;
	r_line->m_pointB = forceThumbLine->m_pointA + r*0.1;*/


	//if (m_numTools > 1)
	//{
	//	forceFingerLine->m_pointA = m_tool1->getHapticPoint(0)->getGlobalPosProxy();
	//	forceFingerLine->m_pointB = forceFingerLine->m_pointA + m_force*0.001;
	//}


	guideWireUpdateMutex.release();

}


//===========================================================================
/*!
    Update haptics.
*/
//===========================================================================
bool contact = false;
cVector3d angle_ref = cVector3d(0.0, 0.0, 0.0);
void cGenericDemo::updateHaptics() 
{ 
	/////////////////////////////////////////////////////////////////////
	// HAPTIC FORCE COMPUTATION
	/////////////////////////////////////////////////////////////////////

    // compute global reference frames for each object
    m_world->computeGlobalPositions(true);

    // update positions and rotations
    for (int i=0; i<m_numTools; i++)
    {
		// update position and orientation of tool
        m_tools[i]->updateFromDevice();
        cMatrix3d rot = m_tools[i]->getDeviceLocalRot();
		
		// doesn't automatically rotate gripper endpoints with gripper body
        m_tools[i]->getHapticPoint(0)->m_sphereProxy->setLocalRot(rot); 
	}   

    // compute interaction forces
	for (int i = 0; i < m_numTools; i++)
	{	
		m_tools[i]->computeInteractionForces();// This function was setting the forces but also making the torque 0

		//my own computeInteractionForces()
		//check out CGenericTool.cpp for original function
		
		//force.zero();
		//torque.zero();

		//int numContactPoint = (int)(m_tools[i]->getNumHapticPoints());
		//for (int j = 0; j<numContactPoint; j++)
		//{
		//	// get next haptic point
		//	cHapticPoint* nextContactPoint = m_tools[i]->getHapticPoint(j);

		//	// compute force at haptic point as well as new proxy position

		//	cVector3d m_deviceGlobalPos = m_tools[i]->getDeviceGlobalPos();
		//	cMatrix3d m_deviceGlobalRot = m_tools[i]->getDeviceGlobalRot();
		//	cVector3d m_deviceGlobalLinVel = m_tools[i]->getDeviceGlobalLinVel();
		//	cVector3d m_deviceGlobalAngVel = m_tools[i]->getDeviceGlobalAngVel();


		//	//******************** FORCE CALCULATION ****************************//
		//	cVector3d t_force = nextContactPoint->computeInteractionForces(m_deviceGlobalPos,
		//		m_deviceGlobalRot,
		//		m_deviceGlobalLinVel,
		//		m_deviceGlobalAngVel);


		//	//******************** TORQUE CALCULATION ****************************//
		//	// combine force contributions together
		//	if (i == 0) r = r0;
		//	else r = r1;

		//	force.add(t_force);
		//	torque.add(cCross(r, t_force));
		//}


		////update global forces
		//m_tools[i]->setDeviceGlobalForce(force);
		//m_tools[i]->setDeviceGlobalTorque(torque);
		//m_tools[i]->setGripperForce(0.0);
	}

	// Soft Finger Proxy For Torque Rendering - Zong
	// check is we are making first contact - test how we can do this easily
	// try checking if the last global force was zero. If it was then we can assume contact was broken?
	contact = (bool) m_force.length();//m_tools[0]->m_hapticPoint->isInContact(m_ode);

	
	if (contact)
	{
		if (!prev_contact) // if we are transition from no contact to contact
		{
			// grab the normal vector
			cVector3d normal_vector = cNormalize(m_tools[0]->m_hapticPoint->m_algorithmFingerProxy->getNormalForce());
			// dot the x-axis of the device frame with the normal
			cMatrix3d m_deviceGlobalRot = m_tools[0]->getDeviceGlobalRot();
			cVector3d proj_normal = cMul(cDot(m_deviceGlobalRot.getCol0(), normal_vector), normal_vector);
			// project into the plane
			cVector3d proj_plane = m_deviceGlobalRot.getCol0() - proj_normal;
			angle_ref = proj_plane; //save the vector as reference
		}
		prev_contact = true;
	}
	else // we are in contact
	{
		prev_contact = false; 
	}

	angle = 0;
	if (contact)
	{
		// grab the normal vector
		cVector3d normal_vector = cNormalize(m_tools[0]->m_hapticPoint->m_algorithmFingerProxy->getNormalForce());
		// dot the x-axis of the device frame with the normal
		cMatrix3d m_deviceGlobalRot = m_tools[0]->getDeviceGlobalRot();
		cVector3d proj_normal = cMul(cDot(m_deviceGlobalRot.getCol0(), normal_vector), normal_vector);
		// project into the plane
		cVector3d proj_plane = m_deviceGlobalRot.getCol0() - proj_normal;
		// compute the other perpendicular vector
		cVector3d perp_vector = cCross(cNormalize(angle_ref), normal_vector);
		// get the sign by projecting into the perp vector
		double sign = cSign(cDot(proj_plane, perp_vector));

		//compute the angle between the reference and the current
		angle = sign*cAngle(proj_plane, angle_ref);

		
	}

	double angle_gain = 1.0;
	m_torque = cVector3d(0,0,angle_gain*angle);
	m_force = m_tools[0]->getHapticPoint(0)->getLastComputedForce();
	m_tools[0]->setDeviceLocalForce(m_force);
	m_tools[0]->setDeviceLocalTorque(m_torque);
	m_tools[0]->applyToDevice();

	/*
    // apply forces to haptic devices
    for (int i=0; i<m_numTools; i++)
    {

		// get global coordinate forces and convert to gripper frame
		m_force = m_tools[i]->getHapticPoint(0)->getLastComputedForce();

		if (i == 0) r = r0;
		else r = r1;
		m_torque.add(cCross(r, m_force));


		// apply force and torque to tools
		m_tools[i]->setDeviceLocalForce(m_force);
		m_tools[i]->setDeviceLocalTorque(m_torque);
		m_tools[i]->applyToDevice();
	}
	*/
    // apply forces to ODE objects
    for (int i=0; i<m_numTools; i++)
    {
        // for each interaction point of the tool we look for any contact events
        // with the environment and apply forces accordingly
        int numInteractionPoints = m_tools[i]->getNumHapticPoints();

        for (int j=0; j<numInteractionPoints; j++)
        {
            // get pointer to next interaction point of tool
            cHapticPoint* interactionPoint = m_tools[i]->getHapticPoint(j);

            // check all contact points
            int numContacts = interactionPoint->getNumCollisionEvents();



            for (int k=0; k<numContacts; k++)
            {
                cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(k);

				

                // given the mesh object we may be touching, we search for its owner which
                // could be the mesh itself or a multi-mesh object. Once the owner found, we
                // look for the parent that will point to the ODE object itself.
                cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

                // cast to ODE object
                cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

                // if ODE object, we apply interaction forces (why the 0.3? this is from the sample code)
                if (ODEobject != NULL)
                {

					ODEobject->addExternalForceAtPoint(-0.3* interactionPoint->getLastComputedForce(),
						collisionEvent->m_globalPos);

					////read position of device
					cVector3d pos_device = m_tools[i]->getHapticPoint(0)->getGlobalPosProxy();

					if ((k == 0)) { //only run tosion algorithm for first collision point
									//************************* GET THE OBJECT POSITION FOR TORQUE CALCULATION *******************************

						object_pos = object->getGlobalPos();
						if (i == 0) r0 = object_pos - pos_device;
						else r1 = object_pos - pos_device;

					}
                }
            }

			
        }
    }


    // retrieve simulation time and compute next interval
    double time = simClock.getCurrentTimeSeconds();
    double nextSimInterval = cClamp(time, 0.0001, 0.001);


    // reset clock
    simClock.reset();
    simClock.start();

    // update simulation
    m_ODEWorld->updateDynamics(nextSimInterval);
	//guideWireUpdateMutex.release();

}


//===========================================================================
/*!
    Set device offset.
*/
//===========================================================================
void cGenericDemo::setOffset(double a_offset)
{
    if (m_tool0 != NULL)
    {
        cVector3d pos(0.0, a_offset, 0.0);
        m_tool0->setLocalPos(pos);
    }
    if (m_tool1 != NULL)
    {
        cVector3d pos(0.0,-a_offset, 0.0);
        m_tool1->setLocalPos(pos);
    }
}


//===========================================================================
/*!
    Set torque gain.
*/
//===========================================================================
void cGenericDemo::setTorqueGain(double a_torqueGain)
{
    m_torqueGain = a_torqueGain;
}


