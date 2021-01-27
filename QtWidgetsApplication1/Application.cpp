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

		string resourceRoot = QCoreApplication::applicationDirPath().toStdString();
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
	stop();
	delete m_demo1;
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
	if (demo_started) {
		if (m_demo->m_tool0 != NULL)
		{
			m_demo->m_tool0->stop();
		}
		if (m_demo->m_tool1 != NULL)
		{
			m_demo->m_tool1->stop();
		}
	}


    // update state
    m_running = false;

    // release run lock
    m_runLock.release();

    // exit thread
    return (NULL);
}

//------------------------------------------------------------------------------

bool ApplicationWidget::getHapticDevice() {
	//--------------------------------------------------------------------------
	// HAPTIC DEVICES / TOOLS
	//--------------------------------------------------------------------------

	// default stiffness of scene objects
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

void ApplicationWidget::getNeutralPos(double &x, double &y, double &z, double &theta) {
	double x_neutralPos, y_neutralPos, z_neutralPos, theta_neutralPos;
	m_hapticDevice0->getNeutralPos(x_neutralPos, y_neutralPos, z_neutralPos, theta_neutralPos);
	x = x_neutralPos;
	y = y_neutralPos;
	z = z_neutralPos;
	theta = theta_neutralPos;

}

void ApplicationWidget::setNeutralPosX(const double &x) {
	m_hapticDevice0->setNeutralPosX(x);
}

void ApplicationWidget::setNeutralPosY(const double &y) {
	m_hapticDevice0->setNeutralPosY(y);
}

void ApplicationWidget::setNeutralPosZ(const double &z) {
	m_hapticDevice0->setNeutralPosZ(z);
}

void ApplicationWidget::setNeutralPosTheta(const double &theta) {
	m_hapticDevice0->setNeutralPosTheta(theta);
}