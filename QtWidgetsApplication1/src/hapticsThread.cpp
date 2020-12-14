#include "hapticsThread.h"


// call in main before starting thread 
void hapticsThread::pairWithGripper(gripper *a_gripper) {
	m_gripper = a_gripper;

}

int hapticsThread::initialize( void ) {
	int errors = 0;
	
	// create Chai device for magnetic tracker
	//chaiMagDevice = chai3d::cGenericHapticDevicePtr((chai3d::cGenericHapticDevice *)(new chai3d::gripperChaiDevice( 0)));

	initializeChai3dStuff();
	m_worldLock.acquire();
	setUpWorld();
	setUpHapticDevice();
	setUpODEWorld();
	ODERings();
	setUpWidgets();
	m_worldLock.release();
	//addObjects();
	
	// set output to zero to start
	m_gripperForce = { 0.0, 0.0, 0.0 };
	m_gripperTorque = { 0.0, 0.0, 0.0 };
	m_gripperGripForce = 0.0;

	// timeout period
	m_timer.setTimeoutPeriodSeconds(0.001);
	m_timer.start();

	return errors;
}

// initialization sub-functions
int hapticsThread::initializeChai3dStuff(void) {
	
	//--------------------------------------------------------------------------
	// OPEN GL - WINDOW DISPLAY
	//--------------------------------------------------------------------------

		// initialize GLFW library
		if (!glfwInit())
		{
			cout << "failed initialization" << endl;
			cSleepMs(1000);
			return 1;
		}


		// set error callback
		//glfwSetErrorCallback(errorCallback);

		// compute desired size of window
		const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
		int w = 0.8 * mode->height;
		int h = 0.5 * mode->height;
		int x = 0.5 * (mode->width - w);
		int y = 0.5 * (mode->height - h);



		// set OpenGL version
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

		if (stereoMode == C_STEREO_ACTIVE)								// set active stereo mode
		{
			glfwWindowHint(GLFW_STEREO, GL_TRUE);
		}
		else
		{
			glfwWindowHint(GLFW_STEREO, GL_FALSE);
		}

		window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);			// create display context

		if (!window)
		{
			cout << "failed to create window" << endl;
			cSleepMs(1000);
			glfwTerminate();
			return 1;
		}

		glfwGetWindowSize(window, &width, &height);						// get width and height of window
		glfwSetWindowPos(window, x, y);									// set position of window
		glfwMakeContextCurrent(window);									// set current display context
		glfwSwapInterval(swapInterval);									// sets the swap interval for the current display context




#ifdef GLEW_VERSION
		// initialize GLEW library
		if (glewInit() != GLEW_OK)
		{
			cout << "failed to initialize GLEW library" << endl;
			glfwTerminate();
			return 1;
		}
#endif



		return 0;
}

int hapticsThread::setUpODEWorld(void) {

	//-----------------------------------------------------------------------
	// CREATE ODE WORLD AND OBJECTS
	//-----------------------------------------------------------------------

	//////////////////////////////////////////////////////////////////////////
	// ODE WORLD
	//////////////////////////////////////////////////////////////////////////

	// create an ODE world to simulate dynamic bodies
	ODEWorld = new cODEWorld(world);

	// add ODE world as a node inside world
	world->addChild(ODEWorld);

	// set some gravity
	ODEWorld->setGravity(cVector3d(0.00, 0.00, -9.81));

	// define damping properties
	ODEWorld->setAngularDamping(0.0002); //( 0.02);
	ODEWorld->setLinearDamping(0.0002);



	//////////////////////////////////////////////////////////////////////////
	// 6 ODE INVISIBLE WALLS
	//////////////////////////////////////////////////////////////////////////

	// we create 6 static walls to contains the 3 cubes within a limited workspace
	ODEGPlane0 = new cODEGenericBody(ODEWorld);
	ODEGPlane1 = new cODEGenericBody(ODEWorld);
	ODEGPlane2 = new cODEGenericBody(ODEWorld);
	ODEGPlane3 = new cODEGenericBody(ODEWorld);
	ODEGPlane4 = new cODEGenericBody(ODEWorld);
	ODEGPlane5 = new cODEGenericBody(ODEWorld);

	int w = 1.0;
	ODEGPlane0->createStaticPlane(cVector3d(0.0, 0.0, 2.0 * w), cVector3d(0.0, 0.0, -1.0));
	ODEGPlane1->createStaticPlane(cVector3d(0.0, 0.0, -w), cVector3d(0.0, 0.0, 1.0));
	ODEGPlane2->createStaticPlane(cVector3d(0.0, w, 0.0), cVector3d(0.0, -1.0, 0.0));
	ODEGPlane3->createStaticPlane(cVector3d(0.0, -w, 0.0), cVector3d(0.0, 1.0, 0.0));
	ODEGPlane4->createStaticPlane(cVector3d(w, 0.0, 0.0), cVector3d(-1.0, 0.0, 0.0));
	ODEGPlane5->createStaticPlane(cVector3d(-0.8 * w, 0.0, 0.0), cVector3d(1.0, 0.0, 0.0));


	//////////////////////////////////////////////////////////////////////////
	// GROUND
	//////////////////////////////////////////////////////////////////////////

	// create a mesh that represents the ground
	cMesh* ground = new cMesh();
	ODEWorld->addChild(ground);

	// create a plane
	double groundSize = 3.0;
	cCreatePlane(ground, groundSize, groundSize);

	// position ground in world where the invisible ODE plane is located (ODEGPlane1)
	ground->setLocalPos(0.0, 0.0, -1.0);

	// define some material properties and apply to mesh
	cMaterial matGround;
	matGround.setStiffness(0.2* maxStiffness);
	matGround.setDamping(0.001*maxStiffness);
	matGround.setDynamicFriction(0.2);
	matGround.setStaticFriction(0.1);
	matGround.setWhite();
	matGround.m_emission.setGrayLevel(0.3);
	ground->setMaterial(matGround);

	// setup collision detector
	ground->createAABBCollisionDetector(toolRadius);


	//////////////////////////////////////////////////////////////////////////
	// Cylinders
	//////////////////////////////////////////////////////////////////////////

	double pegRad = 0.05;
	double pegHeight = 0.4;

	cMesh* cylinder1 = new cMesh();
	ODEWorld->addChild(cylinder1);
	cCreateCylinder(cylinder1, pegHeight, pegRad);
	cylinder1->setMaterial(matGround);
	cylinder1->m_material->setBlack();
	cylinder1->m_material->m_emission.setGrayLevel(0.1);
	cylinder1->setLocalPos(0.0, 0.5, -1.0);
	cylinder1->createAABBCollisionDetector(toolRadius);

	cMesh* cylinder2 = new cMesh();
	ODEWorld->addChild(cylinder2);
	cCreateCylinder(cylinder2, pegHeight, pegRad);
	cylinder2->setMaterial(matGround);
	cylinder2->m_material->setBlack();
	cylinder2->setLocalPos(0.0, -0.5, -1.0);
	cylinder2->createAABBCollisionDetector(toolRadius);

	//cMesh* cylinder3 = new cMesh();
	//ODEWorld->addChild(cylinder3);
	//cCreateCylinder(cylinder3, pegHeight, pegRad);
	//cylinder3->setMaterial(matGround);
	//cylinder3->setLocalPos(-0.5, -0.5, -1.0);
	//cylinder3->createAABBCollisionDetector(toolRadius);

	//cMesh* cylinder4 = new cMesh();
	//ODEWorld->addChild(cylinder4);
	//cCreateCylinder(cylinder4, pegHeight, pegRad);
	//cylinder4->setMaterial(matGround);
	//cylinder4->setLocalPos(-0.5, 0.5, -1.0);
	//cylinder4->createAABBCollisionDetector(toolRadius);

	return 0;
}

int hapticsThread::ODERings(void) {

	// define some material properties for each cube
	cMaterial mat0, mat1, mat2;
	mat0.setRedIndian();
	mat0.setStiffness(0.1 * maxStiffness);
	mat0.setDamping(0.001*maxStiffness);
	mat0.setDynamicFriction(0.7);
	mat0.setStaticFriction(0.8);

	mat1.setBlueRoyal();
	mat1.setStiffness(0.1 * maxStiffness);
	mat1.setDamping(0.01*maxStiffness);
	mat1.setDynamicFriction(0.7);
	mat1.setStaticFriction(0.8);

	////--------------------------------------------------------------------------
	//// SURGICAL BLOCK MESHES
	////--------------------------------------------------------------------------
	//// create a new ODE object that is automatically added to the ODE world
	//ODEBlock0 = new cODEGenericBody(ODEWorld);
	//ODEBlock1 = new cODEGenericBody(ODEWorld);

	//block0 = new chai3d::cMultiMesh(); // create the finger
	//world->addChild(block0);
	//block0->setFrameSize(0.5);
	//block0->setLocalPos(0.5, 0.5, 0.0);

	//block1 = new chai3d::cMultiMesh(); //create the thumb
	//world->addChild(block1);
	//block1->setFrameSize(0.5);
	//block1->setLocalPos(0, 0, 0);

	//// load an object file
	//if (cLoadFileSTL(block0, "../Resources/TrainingBlock.stl")) {
	//	cout << "Block 1 file loaded." << endl;
	//}
	//else {
	//	cout << "Failed to load block model." << endl;
	//}
	//if (cLoadFileOBJ(block1, "../Resources/TrainingBlock.stl")) {
	//	cout << "Block 2 file loaded." << endl;
	//}

	//block0->createAABBCollisionDetector(toolRadius);
	//block1->createAABBCollisionDetector(toolRadius);

	//block0->setMaterial(mat0);
	//block1->setMaterial(mat1);

	//////////////////////////////////////////////////////////////////////////
	// 2 ODE Rings
	//////////////////////////////////////////////////////////////////////////

	// create a new ODE object that is automatically added to the ODE world
	ODERing0 = new cODEGenericBody(ODEWorld);
	//ODERing1 = new cODEGenericBody(ODEWorld);

	// create a virtual mesh  that will be used for the geometry representation of the dynamic body
	cMesh* ring0 = new cMesh();
	//cMesh* ring1 = new cMesh();

	// create a ring mesh
	double innerRad = 0.05;
	cCreateRing(ring0, innerRad, 4.0*innerRad);
	ring0->createAABBCollisionDetector(toolRadius);

	//cCreateRing(ring1, innerRad, 4.0*innerRad);
	//ring1->createAABBCollisionDetector(toolRadius);


	ring0->setMaterial(mat1);
	//ring1->setMaterial(mat1);

	// add mesh to ODE object
	ODERing0->setImageModel(ring0);
	//ODERing1->setImageModel(ring1);

	// create a dynamic model of the ODE object. Here we decide to use a box just like
	// the object mesh we just defined
	// ODERing0->createDynamicBox(size, size, size);
	// ODERing1->createDynamicBox(size, size, size);
	ODERing0->createDynamicMesh();
	//ODERing1->createDynamicMesh();

	// define some mass properties for each cube
	ODERing0->setMass(0.25);
	//ODERing1->setMass(0.00125);

	// set position of each ring
	ODERing0->setLocalPos(0.0, -0.5, -0.90);
	//ODERing1->setLocalPos(-0.5, 0.5, -0.90);
	return 0;
}

int hapticsThread::ODEBoxes(void) {


	////////////////////////////////////////////////////////////////////////////
	//// 3 ODE BLOCKS
	////////////////////////////////////////////////////////////////////////////

	//// create a new ODE object that is automatically added to the ODE world
	//ODEBody0 = new cODEGenericBody(ODEWorld);
	//ODEBody1 = new cODEGenericBody(ODEWorld);
	//ODEBody2 = new cODEGenericBody(ODEWorld);

	//// create a virtual mesh  that will be used for the geometry representation of the dynamic body
	//cMesh* object0 = new cMesh();
	//cMesh* object1 = new cMesh();
	//cMesh* object2 = new cMesh();

	//// create a cube mesh
	//double size = 0.20;
	//cCreateBox(object0, size, size, size);
	//object0->createAABBCollisionDetector(toolRadius);

	//cCreateBox(object1, size, size, size);
	//object1->createAABBCollisionDetector(toolRadius);

	//cCreateBox(object2, size, size, size);
	//object2->createAABBCollisionDetector(toolRadius);

	//// define some material properties for each cube
	//cMaterial mat0, mat1, mat2;
	//mat0.setRedIndian();
	//mat0.setStiffness(0.1 * maxStiffness);
	//mat0.setDynamicFriction(0.6);
	//mat0.setStaticFriction(0.6);
	//object0->setMaterial(mat0);

	//mat1.setBlueRoyal();
	//mat1.setStiffness(0.1 * maxStiffness);
	//mat1.setDynamicFriction(0.6);
	//mat1.setStaticFriction(0.6);
	//object1->setMaterial(mat1);

	//mat2.setGreenDarkSea();
	//mat2.setStiffness(0.1 * maxStiffness);
	//mat2.setDynamicFriction(0.6);
	//mat2.setStaticFriction(0.6);
	//object2->setMaterial(mat2);

	//// add mesh to ODE object
	//ODEBody0->setImageModel(object0);
	//ODEBody1->setImageModel(object1);
	//ODEBody2->setImageModel(object2);

	//// create a dynamic model of the ODE object. Here we decide to use a box just like
	//// the object mesh we just defined
	//ODEBody0->createDynamicBox(size, size, size);
	//ODEBody1->createDynamicBox(size, size, size);
	//ODEBody2->createDynamicBox(size, size, size);

	//// define some mass properties for each cube
	//ODEBody0->setMass(0.00125);
	//ODEBody1->setMass(0.00125);
	//ODEBody2->setMass(0.00125);

	//// set position of each cube
	//ODEBody0->setLocalPos(0.0, -0.6, -3 * 0.5);
	//ODEBody1->setLocalPos(0.0, 0.6, -2 * 0.5);
	//ODEBody2->setLocalPos(0.0, 0.0, -3 * 0.5);

	//// rotate central cube 45 degrees around z-axis
	//ODEBody0->rotateAboutGlobalAxisDeg(0, 0, 1, 45);

	return 0;
}


int hapticsThread::setUpWorld(void) {

	//--------------------------------------------------------------------------
	// WORLD - CAMERA - LIGHTING
	//--------------------------------------------------------------------------

	world = new cWorld();					// create a new world.
	world->m_backgroundColor.setWhite();	// set the background color of the environment
	camera = new cCamera(world);			// create a camera and insert it into the virtual world
	world->addChild(camera);

	// position and orient the camera
	camera->set(cVector3d(2.0, 0.0, 0.2),   // camera position (eye)
		cVector3d(0.0, 0.0, -0.5),			// lookat position (target)
		cVector3d(0.0, 0.0, 1.0));			// direction of the "up" vector

											// set the near and far clipping planes of the camera
											// anything in front or behind these clipping planes will not be rendered
	camera->setClippingPlanes(0.01, 10.0);
	camera->setStereoMode(stereoMode);				// set stereo mode
	camera->setStereoEyeSeparation(0.02);			// set stereo eye separation and focal length (applies only if stereo is enabled)
	camera->setStereoFocalLength(2.0);
	camera->setMirrorVertical(mirroredDisplay);		// set vertical mirrored display mode

	light = new cSpotLight(world);			// create a light source
	camera->addChild(light);				// attach light to camera
	light->setEnabled(true);				// enable light source
	light->setLocalPos(0.0, 0.0, 3.0);		// position the light source
	light->setDir(0.0, 0.0, -1.0);			// define the direction of the light beam
	light->setSpotExponent(0.0);			// set uniform concentration level of light 
	light->setShadowMapEnabled(true);		// enable this light source to generate shadows
	light->m_shadowMap->setQualityLow();			// set the resolution of the shadow map
	//light->m_shadowMap->setQualityMedium();
	light->setCutOffAngleDeg(45);			// set light cone half angle

	return 0;
}

int hapticsThread::setUpHapticDevice(void) {

	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	chaiMagDevice->setEnableGripperUserSwitch(true);
	tool = new cToolGripper(world);					// create a 3D tool and add it to the world
	world->addChild(tool);
	
	cMaterial matFingers;
	matFingers.setStiffness(0.2* maxStiffness);
	matFingers.setDamping(0.001*maxStiffness);
	matFingers.setDynamicFriction(0.2);
	matFingers.setStaticFriction(0.1);

	tool->setHapticDevice(chaiMagDevice);		// connect the haptic device to the tool
	toolRadius = 0.05;							// define the radius of the tool (sphere)
	tool->setRadius(toolRadius);				// define a radius for the tool
	tool->setShowContactPoints(false, false);		// hide the device sphere. only show proxy.
	tool->setShowFrame(false);
	tool->setFrameSize(1.0);
	tool->setWorkspaceRadius(1.3);		// map the physical workspace of the haptic device to a larger virtual workspace.
	tool->setMaterial(matFingers);


	// enable if objects in the scene are going to rotate or translate
	// or possibly collide against the tool. If the environment
	// is entirely static, you can set this parameter to "false"
	tool->enableDynamicObjects(true);


	// haptic forces are enabled only if small forces are first sent to the device;
	// this mode avoids the force spike that occurs when the application starts when 
	// the tool is located inside an object for instance. 
	tool->setWaitForSmallForce(true);

	// initialize tool by connecting to haptic device
	tool->start();


	//tool->m_hapticPointFinger->m_sphereProxy->addChild(m_curSphere0);
	//tool->m_hapticPointThumb->m_sphereProxy->addChild(m_curSphere1);
	tool->setShowEnabled(true, true);
	tool->m_hapticPointFinger->setShow(false, false);
	tool->m_hapticPointThumb->setShow(false, false);


	loadFingerMeshes();
	return 0;
}



void hapticsThread::setUpWidgets(void) {
	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	font = NEW_CFONTCALIBRI20();		//create a font

	// create a label to display the haptic and graphic rate of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(labelRates);

	labelForce = new cLabel(font);
	labelForce->m_fontColor.setWhite();
	camera->m_frontLayer->addChild(labelForce);

}


void hapticsThread::loadFingerMeshes(void) {
	
	//--------------------------------------------------------------------------
	// FINGER MESHES
	//--------------------------------------------------------------------------
	finger = new chai3d::cMultiMesh(); // create the finger
	world->addChild(finger);	
	finger->setFrameSize(0.5);			
	finger->setLocalPos(0.0, 0.0, 0.0);

	thumb = new chai3d::cMultiMesh(); //create the thumb
	world->addChild(thumb);
	thumb->setFrameSize(0.5);
	thumb->setLocalPos(0, 0, 0);

	// load an object file
	if (cLoadFileOBJ(finger, "../../Resources/FingerModel.obj")) {
		cout << "Finger file loaded." << endl;
	}
	else if (cLoadFileOBJ(finger, "../Resources/FingerModel.obj")) {
		cout << "Finger file loaded." << endl;
	}
	else {
		cout << "Failed to load finger model." << endl;
	}
	if (cLoadFileOBJ(thumb, "../../Resources/FingerModelThumb.obj")) {
		cout << "Thumb file loaded." << endl;
	}
	else if (cLoadFileOBJ(thumb, "../Resources/FingerModelThumb.obj")) {
		cout << "Thumb file loaded." << endl;
	}
	else {
		cout << "Failed to load thumb model." << endl;
	}

	// set params for finger
	finger->scale(7);
	finger->setShowEnabled(true);
	finger->setUseVertexColors(true);
	chai3d::cColorf fingerColor;
	fingerColor.setBrownSandy();
	finger->setVertexColor(fingerColor);
	finger->m_material->m_ambient.set(0.1, 0.1, 0.1);
	finger->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	finger->m_material->m_specular.set(1.0, 1.0, 1.0);
	finger->setUseMaterial(true);
	finger->setHapticEnabled(false);
	finger->setShowFrame(false);

	// set params for thumb
	thumb->scale(7);
	thumb->setShowEnabled(true);
	thumb->setUseVertexColors(true);
	chai3d::cColorf thumbColor;
	thumbColor.setBrownSandy();
	thumb->setVertexColor(thumbColor);
	thumb->m_material->m_ambient.set(0.1, 0.1, 0.1);
	thumb->m_material->m_diffuse.set(0.3, 0.3, 0.3);
	thumb->m_material->m_specular.set(1.0, 1.0, 1.0);
	thumb->setUseMaterial(true);
	thumb->setHapticEnabled(false);
	thumb->setShowFrame(false);


}

void hapticsThread::toggleHaptics(void) {
	m_gripper->hapticsOn = !m_gripper->hapticsOn;
	cout << "Haptics: " << m_gripper->hapticsOn << endl;

}

// main thread loop
void hapticsThread::updateHaptics(void)
{
	// reset clock
	cPrecisionClock clock;
	clock.reset();

	// simulation in now running
	simulationRunning = true;
	simulationFinished = false;

	// main haptic simulation loop
	while (simulationRunning)
	{

		if (m_timer.timeoutOccurred())
		{

		/////////////////////////////////////////////////////////////////////
		// SIMULATION TIME
		/////////////////////////////////////////////////////////////////////

		// stop the simulation clock
		clock.stop();

		// read the time increment in seconds
		double timeInterval = clock.getCurrentTimeSeconds();
		//if (timeInterval > 0.002) { timeInterval = 0.002; }		// max out at 500 Hz
		//if (timeInterval < 0.00001) { timeInterval = 0.00001; }		// max out at 500 Hz


		// restart the simulation clock
		clock.reset();
		clock.start();

		// signal frequency counter
		hapticRate.signal(1);

		// update haptic and graphic rate data
		labelRates->setText(cStr(graphicRate.getFrequency(), 0) + " Hz / " +
			cStr(hapticRate.getFrequency(), 0) + " Hz");


		/////////////////////////////////////////////////////////////////////
		// HAPTIC FORCE COMPUTATION
		/////////////////////////////////////////////////////////////////////
		m_worldLock.acquire();
		// compute global reference frames for each object
		world->computeGlobalPositions(true);
		m_worldLock.release();

		m_gripper->m_gripperLock.acquire();
		tool->updateFromDevice();					// update position and orientation of tool
		//cout << tool->getDeviceLocalPos() << endl;
		tool->computeInteractionForces();			// compute interaction forces
		m_gripper->m_gripperLock.release();
		


		/////////////////////////////////////////////////////////////////////
		// SEND FORCES TO HAPTIC GRIPPER
		/////////////////////////////////////////////////////////////////////

		m_gripperForce = tool->getDeviceLocalForce();
		m_gripperTorque = tool->getDeviceLocalTorque();
		m_gripperGripForce = tool->getGripperForce();

		m_gripperRot = tool->getLocalRot();

			// get global coordinate forces and convert to local frame
			m_gripperRot.mulr(tool->m_hapticPointThumb->getLastComputedForce(), m_thumbForce);
			m_gripperRot.mulr(tool->m_hapticPointFinger->getLastComputedForce(), m_fingerForce);
	


		//// send to gripper object
		//m_gripper->m_gripperLock.acquire();
		////cout << m_thumbForce << "   " << m_gripperGripForce << "   ";
		//m_gripper->setForcesAndTorques(m_gripperForce, m_gripperTorque, m_gripperGripForce, m_thumbForce, m_fingerForce);
		//m_gripper->m_gripperLock.release();


		/////////////////////////////////////////////////////////////////////
		// DYNAMIC SIMULATION
		/////////////////////////////////////////////////////////////////////
		//m_worldLock.acquire();
	
		// https://github.com/aleeper/ros_haptics/blob/master/chaifork3/modules/ODE/examples/GLUT/40-ODE-cube/40-ODE-cubic.cpp
		// for each interaction point of the tool we look for any contact events
		// with the environment and apply forces accordingly
		int numInteractionPoints = tool->getNumHapticPoints();
		for (int i = 0; i<numInteractionPoints; i++)
		{
			// get pointer to next interaction point of tool
			cHapticPoint* interactionPoint = tool->getHapticPoint(i);

			// check all contact points
			int numContacts = interactionPoint->getNumCollisionEvents();
			for (int i = 0; i<numContacts; i++)
			{
				cCollisionEvent* collisionEvent = interactionPoint->getCollisionEvent(i);

				// given the mesh object we may be touching, we search for its owner which
				// could be the mesh itself or a multi-mesh object. Once the owner found, we
				// look for the parent that will point to the ODE object itself.
				cGenericObject* object = collisionEvent->m_object->getOwner()->getOwner();

				// cast to ODE object
				cODEGenericBody* ODEobject = dynamic_cast<cODEGenericBody*>(object);

				// if ODE object, we apply interaction forces
				if (ODEobject != NULL)
				{
					ODEobject->addExternalForceAtPoint(-0.3 * interactionPoint->getLastComputedForce(),
						collisionEvent->m_globalPos);
				}
			}

		}

		// update simulation
		ODEWorld->updateDynamics(timeInterval);
		//m_worldLock.release();

		// Output to motors
		m_gripper->motorLoop();

		m_timer.reset();
		m_timer.start();
		}

	}
		

	// exit haptics thread
	simulationFinished = true;
	//m_worldLock.acquire();
	m_gripper->m_gripperLock.acquire();
	m_gripper->disconnect();
	//m_worldLock.release();
	m_gripper->m_gripperLock.release();


}


//------------------------------------------------------------------------------

void hapticsThread::updateGraphics(void)
{	/////////////////////////////////////////////////////////////////////
	// UPDATE WIDGETS
	/////////////////////////////////////////////////////////////////////

	// update haptic and graphic rate data
	 labelRates->setText(cStr(graphicRate.getFrequency(), 0) + " Hz / " +
		cStr(hapticRate.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);

	//// update force rendered
	//labelForce->setText(cStr(&m_gripperForce.x, 0) + ", " + cStr(&m_gripperForce.y, 0) + ", " + cStr(&m_gripperForce.z, 0) + " N" );
	//labelForce->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
	//
	/////////////////////////////////////////////////////////////////////
	// UPDATE FINGER GRAPHICS
	/////////////////////////////////////////////////////////////////////

	finger->setLocalPos(tool->m_hapticPointFinger->getGlobalPosProxy());
	finger->setLocalRot(tool->getDeviceLocalRot());
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 0, 1), 180);
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 90);
	finger->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 180);
	//finger->rotateAboutLocalAxisDeg(cVector3d(-1, 0, 0), (tool->getGripperAngleDeg())/2);

	thumb->setLocalPos(tool->m_hapticPointThumb->getGlobalPosProxy());
	thumb->setLocalRot(tool->getDeviceLocalRot());
	thumb->rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 90);
	//thumb->rotateAboutLocalAxisDeg(cVector3d(1, 0, 0), -(tool->getGripperAngleDeg()) / 2);

	/////////////////////////////////////////////////////////////////////
	// RENDER SCENE
	/////////////////////////////////////////////////////////////////////

	//m_worldLock.acquire();
	// update shadow maps (if any)
	world->updateShadowMaps(false, mirroredDisplay);

	// render world	
	camera->renderView(width, height);                           

	// wait until all GL commands are completed
	glFinish();

	//m_worldLock.release();

	// check for any OpenGL errors
	GLenum err = glGetError();
	if (err != GL_NO_ERROR) cout << "Error: " << gluErrorString(err) << endl;
}


//------------------------------------------------------------------------------
// Helpers
//------------------------------------------------------------------------------

bool hapticsThread::checkSimulationStatus(void) {
	if (simulationFinished) {
		return true;
	}
	else {
		return false;	}
}

//
//void errorCallback(int a_error, const char* a_description)
//{
//	cout << "Error: " << a_description << endl;
//}
