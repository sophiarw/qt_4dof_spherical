#pragma once
#include "chai3d.h"
#include "motorcontrol.h"
#include <cmath>
#include <array>
#include <algorithm>
#include <Eigen/Dense>

#define V_FILT         0.5      // weight for velocity filtering
#define INT_CLMP       0.15      // maximum allowed integrated error
#define PI  3.14159
#define MAX_STRETCH 4.0			// maximum radius of stretch from centerpoint

enum fingers {
	thumb,
	index
};


////DEVICE Parameters (m)
//#define d 0.0175
//#define hi 0.015
//#define d1 0.00497
//#define h1 0.00840
//#define Li 0.0175
//#define li 0.015
//#define xa 0.015
//#define ya 0.015
//#define za 0
//
////Fourbar 
//#define af 0.01205					//input
//#define bf 0.008					//floating
//#define cf 0.007					//output
//#define df 0.010					//ground
//#define angle_offset1  4.76			//offset because motor doesn't sit on same plan as joint
//
//

//DEVICE Parameters (mm)
#define d 17.5
#define hi 17.5
#define d1 4.97
#define h1 8.4
#define Li 17.5
#define li 15
#define xa 14.85
#define ya 14.85
#define za 0

//Fourbar 
#define af 11.95					//input
#define bf 7.35					//floating
#define cf 7.475					//output
#define df 10					//ground
#define angle_offset1 4.76			//degrees offset because motor doesn't sit on same plane as joint



class c4DOFDevice {

private:

public:

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool m_error;						// TRUE = problem with exo while running
	std::string m_errMessage;			// error message
	double m_t;							// current time [sec]
	chai3d::cPrecisionClock* m_clk;		// pointer to clock for computing velocity

	double m_location;


	Eigen::VectorXd m_th;				// current joint angles [rad]
	Eigen::VectorXd m_thDes;			// desired motor joint angles [rad]
	Eigen::VectorXd m_posDes;			// desired end-effector position [mm]
	Eigen::VectorXd m_th_init;			//initial position of motor
	Eigen::VectorXd neutralPos;
	Eigen::VectorXd centerPoint;
	//Changes Sep 20
	// Constructors of c4DOFDevice
	c4DOFDevice();
	~c4DOFDevice();

	void setPos(const Eigen::Ref<Eigen::Vector4d> pos);
	void setForce(const Eigen::Ref<Eigen::Vector4d> a_force);
	void setNeutralPos(const double x, const double y, const double z, const double theta);

	bool m_c4DOFDeviceAvailable;		// TRUE = device instance has been createed
	bool m_c4DOFDeviceReady;			//TRUE =  connection to device successful

	chai3d::cVector3d link1_1;
	chai3d::cVector3d link1_2;
	chai3d::cVector3d link1_3;
	chai3d::cVector3d link1_4;
	chai3d::cVector3d link1_5;
	chai3d::cVector3d link1_6;

	chai3d::cVector3d link2_1;
	chai3d::cVector3d link2_2;
	chai3d::cVector3d link2_3;
	chai3d::cVector3d link2_4;
	chai3d::cVector3d link2_5;
	chai3d::cVector3d link2_6;

	chai3d::cVector3d link3_1;
	chai3d::cVector3d link3_2;
	chai3d::cVector3d link3_3;
	chai3d::cVector3d link3_4;
	chai3d::cVector3d link3_5;
	chai3d::cVector3d link3_6;

	chai3d::cVector3d link4_1;
	chai3d::cVector3d link4_2;
	chai3d::cVector3d link4_3;
	chai3d::cVector3d link4_4;
	chai3d::cVector3d link4_5;
	chai3d::cVector3d link4_6;

	

protected:
	double k_skin_shear = 1.58; // [N/mm] scale force to skin displacement
	double k_skin_normal = 1.58;
	double k_skin_rotational = 1.58; 

	//determine the inverse kinematics based on the stored desired position. Gives JOINT angles
	void inverseKinematics();

	//converts joint angle to motor angle via fourbar transmission to motor
	double fourbar(double angle_i);

	chai3d::cPrecisionClock c4DOFDeviceTimer;


	//constants for ranges
	double xRange = 8.5; 
	double yRange = 12; 
	double zRange_min = -29.0;
	double zRange_max = -16.0;
	double thetaRange = PI / 6;



};



