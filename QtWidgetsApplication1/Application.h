//==============================================================================
/*
    \author    Sophia Williams
*/
//==============================================================================


//------------------------------------------------------------------------------
#ifndef APPLICATION_H
#define APPLICATION_H
//------------------------------------------------------------------------------
#if defined(WIN32) | defined(WIN64)
#pragma warning(disable: 4100)
#endif
//------------------------------------------------------------------------------
#include "chai3d.h"
//------------------------------------------------------------------------------
#include "qt_4dof.h"
//------------------------------------------------------------------------------
#include <QBasicTimer>
#include <QWheelEvent>
#include <QOpenGLWidget>
//------------------------------------------------------------------------------
#include <string>
#include <fstream>
#include <iostream>
#include <random>
#include <stdio.h>
//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
#include "CGenericDemo.h"
#include "CDemo1.h"

#include "c4DOF_chai_device.h"



void _hapticThread (void *arg);

//------------------------------------------------------------------------------

class ApplicationWidget : public QOpenGLWidget
{

    //--------------------------------------------------------------------------
    // CONSTRUCTOR & DESTRUCTOR:
    //--------------------------------------------------------------------------

public:

    ApplicationWidget (QWidget *parent);
    virtual ~ApplicationWidget ();


    //--------------------------------------------------------------------------
    // PUBLIC METHODS:
    //--------------------------------------------------------------------------

public:

    int start();
    int stop();
    void waitForStop();
    void* hapticThread();
    bool isRunning() { return m_running; }

    double getGraphicRate() { return (m_graphicRate.getFrequency()); }
    double getHapticRate() { return  (m_hapticRate.getFrequency()); }

	bool getHapticDevice();




    //--------------------------------------------------------------------------
    // PROTECTED METHODS:
    //--------------------------------------------------------------------------

protected:

    void initializeGL();
    void resizeGL(int a_width, int a_height);
    void paintGL();
    void wheelEvent(QWheelEvent *event);
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);
    void mouseReleaseEvent(QMouseEvent *event);
	void timerEvent(QTimerEvent *event) { update(); }


	//--------------------------------------------------------------------------
	// PRIVATE MEMBERS:
	//--------------------------------------------------------------------------

private: 
	// define the radius of the tool (sphere)
	double toolRadius = 0.008;

    //--------------------------------------------------------------------------
    // PUBLIC MEMBERS:
    //--------------------------------------------------------------------------

public:
	// general variables
	double m_toolRadius;
	double m_maxStiffness;

    // application control
	qt_4dof* m_parent;
    chai3d::cMutex m_worldLock;
    chai3d::cMutex m_runLock;
    chai3d::cThread m_thread;
    chai3d::cFrequencyCounter m_graphicRate;
    chai3d::cFrequencyCounter m_hapticRate;

    int m_timerID;
    bool m_running;
    int m_width;
    int m_height;
    int m_mouseX;
    int m_mouseY;
    bool m_mouseMoveCamera;

    // CHAI3D world
    //chai3d::cGenericHapticDevicePtr m_hapticDevice0;
	//chai3d::cGenericHapticDevicePtr m_hapticDevice1;

	shared_ptr<cGenericHapticDevice> m_hapticDevice0;
	shared_ptr<cGenericHapticDevice> m_hapticDevice1;


	cHapticDeviceHandler* handler;  // a haptic device handler

	//---------------------------------------------------------------------------
	// DEMOS
	//---------------------------------------------------------------------------

	//! currently active camera
	cGenericDemo* m_demo;
	bool demo_started;

	//! Demos
	cDemo1* m_demo1;    // Block Manipulation

	int numDevices;


};

//------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------
