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
#include "CDemo4.h"
using namespace std;
//---------------------------------------------------------------------------

//===========================================================================
/*!
    Constructor of cDemo4.
*/
//===========================================================================
cDemo4::cDemo4(const string a_resourceRoot,
               const int a_numDevices,
               shared_ptr<cGenericHapticDevice> a_hapticDevice0,
               shared_ptr<cGenericHapticDevice> a_hapticDevice1, double maxStiffness):cGenericDemo(a_resourceRoot, a_numDevices, a_hapticDevice0, a_hapticDevice1, maxStiffness)
{
    cMaterial matBase;
    matBase.setGrayLevel(0.3);
    matBase.setStiffness(1500);

	// define some material properties for each cube
	cMaterial mat;
	mat.setWhite();
	mat.m_specular.set(0.0, 0.0, 0.0);
	mat.setStiffness(1000);
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

		// create a new ODE object that is automatically added to the ODE world
		m_ODEBody0 = new cODEGenericBody(m_ODEWorld);
		cMesh* object0 = new cMesh();
		cCreateRing(object0, 0.01, 0.025, 12, 16);
		object0->createAABBCollisionDetector(m_toolRadius);


		object0->setMaterial(mat);
		m_ODEBody0->setImageModel(object0);
		m_ODEBody0->createDynamicMesh();
		m_ODEBody0->setMass(0.02);
		m_ODEBody0->setLocalPos(0.0, 0.0, 0.2);
#endif

	/////////////////////////////////////////////////
	if (0) {
		// add spring to world
		m_ODEWorld->addChild(segments);	// connect some segments to form a spring
		double h = 0.01;
		double dh = 0.0002;
		double a = 0.0;
		double da = 0.2;
		double r = 0.03;
		double x0 = 0.0;
		double y0 = 0.06;
		double z0 = 0.0;

		for (int i = 0; i < 30; i++)
		{
			double px0 = r * cos(a);
			double py0 = r * sin(a);
			double pz0 = h;
			double px1 = r * cos(a + da);
			double py1 = r * sin(a + da);
			double pz1 = h + dh;
			// create vertex 0
			int index0 = segments->newVertex(px0 + x0, py0 + y0, pz0 + z0);

			// create vertex 1
			int index1 = segments->newVertex(px1 + x0, py1 + y0, pz1 + z0);

			// create segment by connecting both vertices together
			segments->newSegment(index0, index1);
			h = h + dh;
			a = a + da;
		}
		// set haptic properties
		segments->setMaterial(mat);
		// assign color properties
		cColorf color;
		color.setYellowGold();
		segments->setLineColor(color);
		// assign line width
		segments->setLineWidth(4.0);
		// use display list for faster rendering
		segments->setUseDisplayList(true);
		// build collision tree
		segments->createAABBCollisionDetector(m_toolRadius);
	}
	//////////////////////////////////////////////////////

	else {
		// add curve to world
		m_ODEWorld->addChild(segments);	// connect some segments to form a spring
		double h = 0.01;
		double dh = 0.00;
		double a = 0.9;
		double da = 0.05;
		double r = 0.15;
		double x0 = 0.0;
		double y0 = 0.0;
		double z0 = -.04;

		for (int i = 0; i < 30; i++)
		{
			double py0 = r * cos(a);
			double pz0 = r * sin(a);
			double px0 = h;
			double py1 = r * cos(a + da);
			double pz1 = r * sin(a + da);
			double px1 = h + dh;
			// create vertex 0
			int index0 = segments->newVertex(px0 + x0, py0 + y0, pz0 + z0);

			// create vertex 1
			int index1 = segments->newVertex(px1 + x0, py1 + y0, pz1 + z0);

			// create segment by connecting both vertices together
			segments->newSegment(index0, index1);
			h = h + dh;
			a = a + da;
		}
		// set haptic properties
		segments->setMaterial(mat);
		// assign color properties
		cColorf color;
		color.setYellowGold();
		segments->setLineColor(color);
		// assign line width
		segments->setLineWidth(4.0);
		// use display list for faster rendering
		segments->setUseDisplayList(true);
		// build collision tree
		//segments->createAABBCollisionDetector(m_toolRadius);

	}

    // initialize
    init();
};


//===========================================================================
/*!
    Set stiffness of environment

    \param  a_stiffness  Stiffness value [N/m]
*/
//===========================================================================
void cDemo4::setStiffness(double a_stiffness)
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
void cDemo4::init()
{
#ifdef ringDemo
    m_ODEBody0->setLocalPos(0.04,-0.05, 0.04);
#endif
}
