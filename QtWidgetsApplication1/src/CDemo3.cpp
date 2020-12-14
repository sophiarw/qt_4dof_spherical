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
#include "CDemo3.h"
#include <random>
#include <chrono>

using namespace std;

//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDemo3.
*/
//===========================================================================
cDemo3::cDemo3(const string a_resourceRoot,
               const int a_numDevices,
               shared_ptr<cGenericHapticDevice> a_hapticDevice0,
               shared_ptr<cGenericHapticDevice> a_hapticDevice1,
				double maxStiffness):cGenericDemo(a_resourceRoot, a_numDevices, a_hapticDevice0, a_hapticDevice1, maxStiffness)
{
    cMaterial matBase;
    matBase.setGrayLevel(0.3);
    matBase.setStiffness(maxStiffness);
	matBase.setTransparencyLevel(0.5);

    double offset = -0.02;
	double width = 0.5;
	double height = 0.5;

    cMatrix3d rot0;
    rot0.identity();
    rot0.rotateAboutGlobalAxisDeg (cVector3d(0,0,1), 45);

    m_ODEBase0 = new cODEGenericBody(m_ODEWorld);
    base0 = new cMesh();
    cCreateBox(base0, 0.005,width, height);
    m_ODEBase0->createDynamicBox(0.005, width, height, true);
    base0->createAABBCollisionDetector(m_toolRadius);
	base0->setUseTransparency(true);
    base0->setMaterial(matBase);
	base0->setShowEnabled(false);
	base0->setHapticEnabled(true);
    m_ODEBase0->setImageModel(base0);
    m_ODEBase0->setLocalPos(0.0+ offset, 0.0 + offset, 0.050);
    m_ODEBase0->setLocalRot(rot0);

	//cMatrix3d rot1;
	//rot1.identity();
	//rot1.rotateAboutGlobalAxisDeg(cVector3d(0, 0, 1), 45);
	//rot1.rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 45);

	//m_ODEBase1 = new cODEGenericBody(m_ODEWorld);
	//cMesh* base1 = new cMesh();
	//cCreateBox(base1, 0.005, width, height);
	//m_ODEBase1->createDynamicBox(0.005, width, height, true);
	//base1->createAABBCollisionDetector(m_toolRadius);
	//base1->setUseTransparency(true);
	//base1->setMaterial(matBase);
	//m_ODEBase1->setImageModel(base0);
	//m_ODEBase1->setLocalPos(0.0 - 2*offset, 0.0 - 2*offset, 0.050);
	//m_ODEBase1->setLocalRot(rot1);
 
    // initialize
    init();
};


//===========================================================================
/*!
    Set stiffness of environment

    \param  a_stiffness  Stiffness value [N/m]
*/
//===========================================================================
void cDemo3::setStiffness(double a_stiffness)
{
    // set ground
   // m_ground->setStiffness(a_stiffness, true);
    
    // set wall
    m_ODEBase0->m_imageModel->setStiffness(a_stiffness);

};


//===========================================================================
/*!
    Initialize position of objects
*/
//===========================================================================
void cDemo3::init()
{
	m_ground->setEnabled(false);
	m_ground->setStiffness(0.0);
	m_ground->setShowEnabled(false);
	m_ground->setHapticEnabled(false);
}


void cDemo3::setVirtualWallPosition() {
	m_ground->setEnabled(false);

	unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
	std::default_random_engine generator(seed);
	std::normal_distribution<double> distribution(0.0, 1.0);
	cout << distribution(generator) << endl;
	cMatrix3d rot0;
	rot0.identity();
	// first rotate about z axis maybe
	if (distribution(generator) > 0.0) {
		rot0.rotateAboutGlobalAxisDeg(cVector3d(0, 0, 1), 60.0 * distribution(generator));
	}
	cout << distribution(generator) << endl;

	// then rotate about y axis maybe
	seed = std::chrono::system_clock::now().time_since_epoch().count();
	if (distribution(generator) > 0.0) {
		rot0.rotateAboutLocalAxisDeg(cVector3d(0, 1, 0), 60.0 * distribution(generator));
	}
	cout << distribution(generator) << endl;

	m_ODEBase0->setLocalPos(distribution(generator)*.1, distribution(generator)*.1, 0.050);
	m_ODEBase0->setLocalRot(rot0);
}