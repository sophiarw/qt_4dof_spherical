//==============================================================================
/*
    \author    Your Name
*/
//==============================================================================

//------------------------------------------------------------------------------
#include "Application.h"
//------------------------------------------------------------------------------
#include <QFile>
#include <QString>
#include <QMessageBox>



ApplicationWidget::ApplicationWidget (QWidget *parent)
{
    //--------------------------------------------------------------------------
    // INITIALIZATION
    //--------------------------------------------------------------------------

    // initialize variables
    m_parent  = (qt_4dof*)(void*)parent;
    m_running = false;
    m_timerID = 0;
    m_mouseMoveCamera = false;

    // reset frequency counters
    m_graphicRate.reset();
    m_hapticRate.reset();



	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------
	getHapticDevice();


	//check if there are devices and environment can be generated
	demo_started = false;
	if (numDevices <= 0) {
		QMessageBox::information(this, "Application", "Cannot start application because no devices are connected.", QMessageBox::Ok);
		stop(); //stop running the application 
	}
	else {
		//--------------------------------------------------------------------------
		// OBJECTS
		//--------------------------------------------------------------------------
		// default stiffness of scene objects
		double maxStiffness = 6000.0;
		double maxDamping = 4.0;

		string resourceRoot = "C:\\Users\\sophi\\source\\repos\\QtWidgetsApplication1";//QCoreApplication::applicationDirPath().toStdString();
		m_demo1 = new cDemo1(resourceRoot, numDevices, m_hapticDevice0, m_hapticDevice1, maxStiffness);
		if (NULL != m_demo1) {
			m_demo = m_demo1;
			m_demo->init();
			m_demo->hapticsOn = true;
			demo_started = true;
		}
	}



};

//------------------------------------------------------------------------------

ApplicationWidget::~ApplicationWidget ()
{
    delete m_world;
	delete m_demo1;
	delete handler;
}

//------------------------------------------------------------------------------

void* ApplicationWidget::hapticThread ()
{
    // acquire run lock
    m_runLock.acquire();

    // update state
    m_running = true;

    while (m_running) 
    {
		m_demo->updateHaptics();

        // update frequency counter
        m_hapticRate.signal(1);
    }

    // disable forces
    m_hapticDevice0->setForceAndTorqueAndGripperForce (cVector3d(0.0, 0.0, 0.0),
                                                cVector3d(0.0, 0.0, 0.0),
                                                0.0);

    // update state
    m_running = false;

    // release run lock
    m_runLock.release();

    // exit thread
    return (NULL);
}


//------------------------------------------------------------------------------

void ApplicationWidget::makeWorld() {
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
		0,      // spherical coordinate azimuth angle
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
}

//------------------------------------------------------------------------------

bool ApplicationWidget::getHapticDevice() {
	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	// create a haptic device handler
	cHapticDeviceHandler* handler = new cHapticDeviceHandler();

	numDevices = handler->getNumDevices();

	// default stiffness of scene objects
	double maxStiffness = 6000.0;
	double maxDamping = 4.0;

	// get access to the haptic devices found
	if (numDevices > 0)
	{
		handler->getDevice(m_hapticDevice0, 0);
		//maxStiffness = cMin(maxStiffness, 0.5 * m_hapticDevice0->getSpecifications().m_maxLinearStiffness);
	}


	else if (numDevices > 1)
	{
		handler->getDevice(m_hapticDevice1, 1);
		//maxStiffness = cMin(maxStiffness, 0.5 * m_hapticDevice1->getSpecifications().m_maxLinearStiffness);
	}

	m_hapticDevice1 = m_hapticDevice0; // only one device for me, figure out what's going on later

	return handler->getNumDevices();

}

//------------------------------------------------------------------------------

void ApplicationWidget::createObjects() {
	// create a sphere
	m_object = new cMultiMesh();

	// add object to world
	m_world->addChild(m_object);

	// position object
	m_object->setLocalPos(0.0, 0.0, 0.0);

	// create a first ring (red)
	cMesh* mesh1 = m_object->newMesh();
	cCreatePipe(mesh1, 0.1, 0.80, 1.00, 72, 1, cVector3d(-0.05, 0.0, 0.0), cMatrix3d(cDegToRad(0), cDegToRad(90), cDegToRad(0), C_EULER_ORDER_XYZ));
	mesh1->m_material->setOrangeRed();
	mesh1->m_material->setStiffness(100.0);

	// create a second ring (green)
	cMesh* mesh2 = m_object->newMesh();
	cCreatePipe(mesh2, 0.1, 0.79, 1.01, 72, 1, cVector3d(0.0, -0.05, 0.0), cMatrix3d(cDegToRad(-90), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ));
	mesh2->m_material->setGreenForest();
	mesh2->m_material->setStiffness(100.0);

	// create a third ring (blue)
	cMesh* mesh3 = m_object->newMesh();
	cCreatePipe(mesh3, 0.1, 0.78, 1.02, 72, 1, cVector3d(0.0, 0.0, -0.05), cMatrix3d(cDegToRad(0), cDegToRad(0), cDegToRad(0), C_EULER_ORDER_XYZ));
	mesh3->m_material->setBlueRoyal();
	mesh3->m_material->setStiffness(100.0);

	// create collision detector
	m_object->createAABBCollisionDetector(toolRadius);
}

//------------------------------------------------------------------------------

void ApplicationWidget::createWidget() {
	// create a background
	cBackground* background = new cBackground();
	m_camera->m_backLayer->addChild(background);

	// set background properties
	background->setCornerColors(cColorf(0.00f, 0.00f, 0.00f),
		cColorf(0.00f, 0.00f, 0.00f),
		cColorf(0.44f, 0.44f, 0.88f),
		cColorf(0.44f, 0.44f, 0.88f));
}

//------------------------------------------------------------------------------

void ApplicationWidget::initializeGL ()
{
#ifdef GLEW_VERSION
    glewInit ();
#endif
}

//------------------------------------------------------------------------------

void ApplicationWidget::paintGL ()
{
    if (!m_running) return;

    m_worldLock.acquire();

    // render world
    //m_camera->renderView(m_width, m_height);

	if (demo_started) {
		m_demo->updateGraphics(m_width, m_height);

		// wait until all GL commands are completed
		glFinish();

		m_graphicRate.signal(1);
	}

   

    m_worldLock.release();
}

//------------------------------------------------------------------------------

void ApplicationWidget::resizeGL (int a_width,  int a_height)
{
    m_worldLock.acquire ();

    m_width = a_width;
    m_height = a_height;

    m_worldLock.release ();
}

//------------------------------------------------------------------------------

int ApplicationWidget::start ()
{
    // start graphic rendering
    m_timerID = startTimer(20);

    // start haptic thread
    m_thread.start (_hapticThread, CTHREAD_PRIORITY_HAPTICS, this);

    return(0);
}

//------------------------------------------------------------------------------

int ApplicationWidget::stop ()
{
    // stop the simulation thread and wait it to join
    m_running = false;
    m_runLock.acquire();
    m_runLock.release();

	if (demo_started) {
		m_demo->m_tool0->stop();
		//m_demo->m_tool1->stop();
	}
    

    killTimer (m_timerID);
	disconnectFromS826();

    return 0;
}

//------------------------------------------------------------------------------

void ApplicationWidget::wheelEvent (QWheelEvent *event)
{
    double radius = m_demo->m_camera->getSphericalRadius() + (double)(event->delta())*5e-4;
	m_demo->m_camera->setSphericalRadius(radius);

    // tell GUI widgets to reflect the new simulation parameters
    m_parent->SyncUI ();
}

//---------------------------------------------------------------------------

void ApplicationWidget::mousePressEvent(QMouseEvent *event)
{
    m_mouseX = event->pos().x();
    m_mouseY = event->pos().y();
    m_mouseMoveCamera = true;
}

//---------------------------------------------------------------------------

void ApplicationWidget::mouseMoveEvent(QMouseEvent *event)
{
    if (m_mouseMoveCamera)
    {
        int x = event->pos().x();
        int y = event->pos().y();

        // compute mouse motion
        int dx = x - m_mouseX;
        int dy = y - m_mouseY;
        m_mouseX = x;
        m_mouseY = y;

        // compute new camera angles
        double azimuthDeg = m_demo->m_camera->getSphericalAzimuthDeg() + (0.5 * dy);
        double polarDeg = m_demo->m_camera->getSphericalPolarDeg() + (-0.5 * dx);

        // assign new angles
		m_demo->m_camera->setSphericalAzimuthDeg(azimuthDeg);
		m_demo->m_camera->setSphericalPolarDeg(polarDeg);

        // line up tool with camera
        m_demo->m_tool0->setLocalRot(m_demo->m_camera->getLocalRot());
    }
}

//---------------------------------------------------------------------------

void ApplicationWidget::mouseReleaseEvent(QMouseEvent *event)
{
    m_mouseMoveCamera = false;
}

//------------------------------------------------------------------------------

void _hapticThread (void *arg)
{
    ((ApplicationWidget*)arg)->hapticThread();
}

//------------------------------------------------------------------------------
