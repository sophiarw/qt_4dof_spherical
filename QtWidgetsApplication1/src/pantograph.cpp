#include "pantograph.h"

using namespace std;
using namespace chai3d;


pantograph::pantograph() {

	// device available for connection
	m_pantographAvailable = true;
	m_error = false;
	m_errMessage = "";

	lpfX.setCutOffFrequency(cutOffFreqFinger);
	lpfZ.setCutOffFrequency(cutOffFreqFinger);

	// initialize kinematic variables
	m_t = 0;
	m_th = cVector3d(0.8796, 2.2620, 0.0);
	m_thDes = cVector3d(0.8796, 2.2620, 0.0);

	centerPoint.set(-(len[4] / 2.0), 0.0, 19.0); //[mm]				// pantograph x positive to left when looking at motor axle, z positive down
	pantographTimer.start();
	
	m_pantographReady = true;

}

pantograph::~pantograph(void) {
}


void pantograph::setPos(cVector3d a_force) {
	if (m_finger == index) {
		a_force.x(-a_force.x());		// x axis directions flipped for one finger
		//cout << endl;
	}
	//if (abs(a_force.x()) > THRESH || abs(a_force.z()) > THRESH) {

		// pangtograph x positive toward left, z positive down (when looking at palm with fingers open)
		cVector3d stretch(a_force.x() / k_skin, 0.0, -a_force.z() / k_skin);

		// keep within center of workspace
		if (stretch.x() > MAX_STRETCH) { stretch.x(MAX_STRETCH); }
		if (stretch.x() < -MAX_STRETCH) { stretch.x(-MAX_STRETCH); }
		if (stretch.z() > MAX_STRETCH) { stretch.z(MAX_STRETCH); }
		if (stretch.z() < -MAX_STRETCH) { stretch.z(-MAX_STRETCH); }

		float deltaT = pantographTimer.getCurrentTimeSeconds();
		m_posDes.x(lpfX.update(centerPoint.x() + stretch.x(), deltaT));
		m_posDes.z(lpfZ.update(centerPoint.z() + stretch.z(), deltaT));
		pantographTimer.reset();
		//cout << m_finger << " stretch: " << m_posDes.x() << ", " << m_posDes.z() << "      " ;
	//}
	//else {
	//	m_posDes.x(centerPoint.x());
	//	m_posDes.z(centerPoint.z());
	//}
	inverseKinematics();
	//cout << endl;
}

void pantograph::inverseKinematics() {
	//calculate desired angles from matlab code
	double xx = m_posDes.x();
	double yy = m_posDes.z();

	// Distances to point from each motor hub
	//double P1P3 = sqrt( (double)(m_pos.x)*(double)(m_pos.x) + (double)(m_pos.z)*(double)(m_pos.z));
	double P1P3 = sqrt(xx*xx + yy*yy);
	double P5P3 = sqrt((xx + len[4])*(xx + len[4]) + yy*yy);
	if ((P1P3 > (len[0] + len[1])) || (P5P3 > (len[3] + len[2]))) {	// longer than length of arms
		return;
	}

	// intermediate terms
	double alpha1 = acos((len[1] * len[1] - len[0] * len[0] - P1P3*P1P3) / (-2 * len[0] * P1P3));
	if (alpha1 < 0) {
		return;
	}

	double beta1 = atan2(yy, -xx);
	double beta5 = acos((len[2] * len[2] - len[3] * len[3] - P5P3*P5P3) / (-2 * len[3] * P5P3));

	if (beta5 < 0) {
		return;
	}
	double alpha5 = atan2(yy, xx + len[4]);

	// set angles in thDes
	m_thDes.set(PI - alpha1 - beta1, alpha5 + beta5, 0.0);

}



double pantograph::angleDiff(double a_thA, double a_thB)
{
	// determine shortest distance between A and B
	double diff1 = fabs(a_thA - a_thB);
	double diff2 = 2 * PI - diff1;
	double diff = fmin(diff1, diff2);

	// determine direction for moving from B to A along shortest path
	if (abs(fmod(a_thB + diff, 2 * PI) - a_thA) < THRESH)  return  1.0*diff;
	else                                                 return -1.0*diff;
}

cVector3d pantograph::vecDiff(cVector3d a_vecA, cVector3d a_vecB)
{
	cVector3d diff;
	for (int i = 0; i < 3; i++) {
		diff(i) = angleDiff(a_vecA(i), a_vecB(i));
	}
	return diff;
}

