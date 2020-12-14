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
    \version   3.2.0 $Rev: 2007 $
*/
//===========================================================================

//---------------------------------------------------------------------------
#include "chai3d.h"
#include <fstream>
#include <iostream>
#include <random>
#include <stdio.h>
#include <string>


//------------------------------------------------------------------------------
#include <GLFW/glfw3.h>
//---------------------------------------------------------------------------
using namespace chai3d;
using namespace std;
//---------------------------------------------------------------------------
#include "CODE.h"
#include "CGenericDemo.h"
#include "CDemo1.h"


//---------------------------------------------------------------------------
//#define JULIE_GRIPPER_DEVICE_SUPPORT 1
//
#include "c4DOF_chai_device.h"
//

//------------------------------------------------------------------------------
// GENERAL SETTINGS
//------------------------------------------------------------------------------

// stereo Mode
/*
    C_STEREO_DISABLED:            Stereo is disabled
    C_STEREO_ACTIVE:              Active stereo for OpenGL NVDIA QUADRO cards
    C_STEREO_PASSIVE_LEFT_RIGHT:  Passive stereo where L/R images are rendered next to each other
    C_STEREO_PASSIVE_TOP_BOTTOM:  Passive stereo where L/R images are rendered above each other
*/
cStereoMode stereoMode = C_STEREO_DISABLED;

// fullscreen mode
bool fullscreen = false;

// mirrored display
bool mirroredDisplay = false;


//---------------------------------------------------------------------------
// CHAI3D VARIABLES
//---------------------------------------------------------------------------

// a haptic device handler
cHapticDeviceHandler* handler;

// a pointer to the current haptic device
shared_ptr<cGenericHapticDevice> m_hapticDevice0(nullptr);
shared_ptr<cGenericHapticDevice> m_hapticDevice1(nullptr);

cPrecisionClock m_clk;         // pointer to clock 


//---------------------------------------------------------------------------
// DEMOS
//---------------------------------------------------------------------------

//! currently active camera
cGenericDemo* m_demo;

//! Demos
cDemo1* m_demo1;    // Block Manipulation



//---------------------------------------------------------------------------
// GENERAL VARIABLES
//---------------------------------------------------------------------------

// flag to indicate if the haptic simulation currently running
bool simulationRunning = false;

// flag to indicate if the haptic simulation has terminated
bool simulationFinished = true;

// a frequency counter to measure the simulation graphic rate
cFrequencyCounter freqCounterGraphics;

// a frequency counter to measure the simulation haptic rate
cFrequencyCounter freqCounterHaptics;

// a label to display the rates [Hz] at which the simulation is running
cLabel* labelRates;


// haptic thread
cThread* hapticsThread;

// a handle to window display context
GLFWwindow* window = NULL;

// current width of window
int width = 0;

// current height of window
int height = 0;

// swap interval for the display context (vertical synchronization)
int swapInterval = 1;


//---------------------------------------------------------------------------
// DECLARED FUNCTIONS
//---------------------------------------------------------------------------

// callback when the window display is resized
void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height);

// callback when an error GLFW occurs
void errorCallback(int error, const char* a_description);

// callback when a key is pressed
void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods);

// this function renders the scene
void updateGraphics(void);

// this function contains the main haptics simulation loop
void updateHaptics(void);

// this function prints position and force data to a file
void recordData(ofstream& file);

// this function closes the application
void close(void);

// initialize demos
void initDemo1();

// trial data recording
int trialNaive = 0;
int trialLQR = 0;
ofstream file;
string filename;
bool recording = false;

//===========================================================================
/*
    DEMO:    06-ODE-explorations.cpp

    This example illustrates the use of the ODE framework for simulating
    haptic interaction with dynamic bodies. 
 */
//===========================================================================

int main(int argc, char* argv[])
{
	//-----------------------------------------------------------------------
	// INITIALIZATION
	//-----------------------------------------------------------------------

	cout << endl;
	cout << "-----------------------------------" << endl;
	cout << "CHAI3D" << endl;
	cout << "Demo: 06-ODE-exploration" << endl;
	cout << "Copyright 2003-2016" << endl;
	cout << "-----------------------------------" << endl << endl << endl;
	cout << "Which program would you like?" << endl << endl;

	cout << "[1] - Select Demo 1: Block Manipulation" << endl;

	int programNum;
	cout << "Selection: ";
	cin >> programNum;
	//programNum = 5;
	while (programNum > 6) {
		cout << "Choose 1-6: ";
		cin >> programNum;
	}

	cout << "Keyboard Options: " << endl;
	cout << "[q] - Exit application" << endl;
	cout << "[f] - Toggle fullscreen" << endl;
	cout << "[r] - Start recording" << endl;
	cout << "[t] - Stop recording" << endl;

	cout << endl << endl;


	// set up file to record 
	double deltaTrecord = 0.01;
	double recordTimeLast = 0.0;

	srand(time(NULL));
	filename = "outputData" + std::to_string(rand()) + ".csv";
	ofstream file;
	file.open(filename);

	// parse first arg to try and locate resources
	string resourceRoot = string(argv[0]).substr(0, string(argv[0]).find_last_of("/\\") + 1);

	// start clock
	m_clk.start(true);


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
	glfwSetErrorCallback(errorCallback);

	// compute desired size of window
	const GLFWvidmode* mode = glfwGetVideoMode(glfwGetPrimaryMonitor());
	int w = 0.8 * mode->height;
	int h = 0.5 * mode->height;
	int x = 0.5 * (mode->width - w);
	int y = 0.5 * (mode->height - h);

	// set OpenGL version
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 1);

	// set active stereo mode
	if (stereoMode == C_STEREO_ACTIVE)
	{
		glfwWindowHint(GLFW_STEREO, GL_TRUE);
	}
	else
	{
		glfwWindowHint(GLFW_STEREO, GL_FALSE);
	}

	// create display context
	window = glfwCreateWindow(w, h, "CHAI3D", NULL, NULL);
	if (!window)
	{
		cout << "failed to create window" << endl;
		cSleepMs(1000);
		glfwTerminate();
		return 1;
	}

	// get width and height of window
	glfwGetWindowSize(window, &width, &height);

	// set position of window
	glfwSetWindowPos(window, x, y);

	// set key callback
	glfwSetKeyCallback(window, keyCallback);

	// set resize callback
	glfwSetWindowSizeCallback(window, windowSizeCallback);

	// set current display context
	glfwMakeContextCurrent(window);

	// sets the swap interval for the current display context
	glfwSwapInterval(swapInterval);
	
	// initialize GLEW library
#ifdef GLEW_VERSION
	if (glewInit() != GLEW_OK)
	{
		cout << "failed to initialize GLEW library" << endl;
		glfwTerminate();
		return 1;
	}
#endif

	//-----------------------------------------------------------------------
	// HAPTIC DEVICES 
	//-----------------------------------------------------------------------

	// create a haptic device handler
	handler = new cHapticDeviceHandler();

	//// get number of haptic devices
	int numDevices = handler->getNumDevices();

	cout << "Num Devices: " << numDevices  << endl;
	//handler->update();


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
	//-----------------------------------------------------------------------
	// DEMOS
	//-----------------------------------------------------------------------

	// setup demo and initialize
	if (programNum == 1) { m_demo1 = new cDemo1(resourceRoot, numDevices, m_hapticDevice0, m_hapticDevice1, maxStiffness);     initDemo1();	}
	else {
		cout << " No program selected! " << endl;
	}



	//--------------------------------------------------------------------------
	// WIDGETS
	//--------------------------------------------------------------------------

	// create a font
	cFontPtr font = NEW_CFONTCALIBRI20();

	// create a label to display the haptic and graphic rates of the simulation
	labelRates = new cLabel(font);
	labelRates->m_fontColor.setBlack();
	m_demo->m_camera->m_frontLayer->addChild(labelRates);



    //-----------------------------------------------------------------------
    // START SIMULATION
    //-----------------------------------------------------------------------

    // create a thread which starts the main haptics rendering loop
    hapticsThread = new cThread();
    hapticsThread->start(updateHaptics, CTHREAD_PRIORITY_HAPTICS);

    // setup callback when application exits
    atexit(close);


    //--------------------------------------------------------------------------
    // MAIN GRAPHIC LOOP
    //--------------------------------------------------------------------------

    // call window size callback at initialization
    windowSizeCallback(window, width, height);

    // main graphic loop
    while (!glfwWindowShouldClose(window))
    {
        // get width and height of window
        glfwGetWindowSize(window, &width, &height);

        // render graphics
        updateGraphics();

        // swap buffers
        glfwSwapBuffers(window);

        // process events
        glfwPollEvents();

		// record data in file
		if (recording && (m_clk.getCurrentTimeSeconds() - recordTimeLast > deltaTrecord)) {
			recordData(file);
			recordTimeLast = m_clk.getCurrentTimeSeconds();
		}
    }

    // close window
    glfwDestroyWindow(window);

    // terminate GLFW library
    glfwTerminate();

    // exit
    return 0;
}

//---------------------------------------------------------------------------

void initDemo1()
{
	if (NULL != m_demo1) {
		m_demo = m_demo1;
		m_demo->init();
		m_demo->hapticsOn = true;
	}

}

//---------------------------------------------------------------------------


//---------------------------------------------------------------------------

void windowSizeCallback(GLFWwindow* a_window, int a_width, int a_height)
{
    // update window size
    width  = a_width;
    height = a_height;
}

//------------------------------------------------------------------------------

void errorCallback(int a_error, const char* a_description)
{
    cout << "Error: " << a_description << endl;
}

//---------------------------------------------------------------------------

void keyCallback(GLFWwindow* a_window, int a_key, int a_scancode, int a_action, int a_mods)
{
	double t_start;
	double t_end1 = 0.25;
	double period1 = t_end1 * 4.0;
	double t_end2 = 1.0;
	double period2 = t_end2 * 4.0;

	double mag = 0.0005; // [mm]
	double X1, Z1, X2, Z2;

	// filter calls that only include a key press
	if ((a_action != GLFW_PRESS) && (a_action != GLFW_REPEAT))
	{
		return;
	}

	// option - exit
	else if ((a_key == GLFW_KEY_ESCAPE) || (a_key == GLFW_KEY_Q))
	{
		glfwSetWindowShouldClose(a_window, GLFW_TRUE);
		if (file.is_open()) {
			file.close();
		}
	}

	// option - start demo 1
	else if (a_key == GLFW_KEY_1)
	{
		initDemo1();
	}

	else if (a_key == GLFW_KEY_R) {
		recording = true;
	}


}

//---------------------------------------------------------------------------

void close(void)
{
    // stop the simulation
    simulationRunning = false;

	disconnectFromS826();

    // wait for graphics and haptics loops to terminate
    while (!simulationFinished) { cSleepMs(100); }

    // delete resources
    delete hapticsThread;
    delete m_demo1;
    delete handler;
}

//---------------------------------------------------------------------------

void updateGraphics(void)
{
    // render world
    m_demo->updateGraphics(width, height);

    // wait until all GL commands are completed
    glFinish();

    // check for any OpenGL errors
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) printf("Error:  %s\n", gluErrorString(err));

	// Frequency labels
	// update haptic and graphic rate data
	labelRates->setText("Graphics: " + cStr(freqCounterGraphics.getFrequency(), 0) + " Hz / Haptics: " +
		cStr(freqCounterHaptics.getFrequency(), 0) + " Hz");

	// update position of label
	labelRates->setLocalPos((int)(0.5 * (width - labelRates->getWidth())), 15);
	// signal frequency counter
	freqCounterGraphics.signal(1);
}

//---------------------------------------------------------------------------

void updateHaptics(void)
{
    // simulation in now running
    simulationRunning = true;
    simulationFinished = false;

    // main haptic simulation loop
    while(simulationRunning)
    {
		m_demo->updateHaptics();
		
		// signal frequency counter
		freqCounterHaptics.signal(1);
    }

    // shutdown haptic devices
    if (m_demo->m_tool0 != NULL)
    {
        m_demo->m_tool0->stop();
    }
    if (m_demo->m_tool1 != NULL)
    {
        m_demo->m_tool1->stop();
    }

    // exit haptics thread
    simulationFinished = true;
}

//---------------------------------------------------------------------------

void recordData(ofstream& file) {

	// retrieve position and orientation information
	cVector3d pos = (m_demo->m_tool0->getDeviceGlobalPos());
	cVector3d torque = (m_demo->m_torque);
	file << m_clk.getCurrentTimeSeconds() << ", ";
	file << pos.x() << ", " << pos.y() << ", " << pos.z() << ", ";
	file << torque.x() << ", " << torque.y() << ", " << torque.z() << ", ";



	file << endl;
}
