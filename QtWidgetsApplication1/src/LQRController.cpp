#include "LQRController.h"

LQRController::LQRController(void) {
	m_xErr = 0.0;
	m_zErr = 0.0;
	m_yawErr = 0.0;
	m_rollErr = 0.0;
	m_pos.set(0.0, 0.0, 0.0);
	m_vel.set(0.0, 0.0, 0.0);
	m_euler.set(0.0, 0.0, 0.0);
	m_rotvel.set(0.0, 0.0, 0.0);
	m_uvw.set(0.0, 0.0, 0.0);
	m_uvwDot.set(0.0, 0.0, 0.0);
	m_pqr.set(0.0, 0.0, 0.0);
	m_pqrDot.set(0.0, 0.0, 0.0);

	// set up filters;
	yprFilter.setCutOffFreq(cutOffFreq);
	rotVelFilter.setCutOffFreq(cutOffFreq);
	linVelFilter.setCutOffFreq(cutOffFreq);

	// fill state vector 
	state << m_xErr, m_uvw(0), m_uvwDot(0), m_zErr, m_uvw(2), m_uvwDot(2), m_rollErr, m_pqr(0), m_pqrDot(0), m_yawErr, m_pqr(2), m_pqrDot(2);

	// fill command vector
	u << 0.0, 0.0, 0.0, 0.0;

	// start clock
	m_clk = new cPrecisionClock();
	m_clk->start();
	last_time = 0.0;

	// load initial gain matrix
	readGainMatrix("../K_gains_julie.csv");

}

 void LQRController::updateAll(cVector3d a_pos, cVector3d a_euler, double a_xErr, double a_zErr, double a_rollErr, double a_yawErr) {
	deltaT = m_clk->getCurrentTimeSeconds() - last_time;
	setRot(a_euler);	// updates all rotation/angular velocity/acceleration components
	setPos(a_pos);		// updates all position/linear velocity/acceleration components
	last_time += deltaT;

	m_xErr = a_xErr;
	m_zErr = a_zErr;
	m_rollErr = a_rollErr; 
	m_yawErr = a_yawErr;

	// update state vector
	state << m_xErr, m_uvw(0), m_uvwDot(0), m_zErr, m_uvw(2), m_uvwDot(2), m_rollErr, m_pqr(0), m_pqrDot(0), m_yawErr, m_pqr(2), m_pqrDot(2);
	
	// read updated K matrix
	readGainMatrix("../K_gains_julie.csv");

	// calculate output pad velocities
	calculateCommand();
}

/* This function opens the text file containing the gain matrix and updates the gain matrix K*/
void LQRController::readGainMatrix(std::string file) {

	std::ifstream in(file);
	std::string line;

	if (in.is_open()) {

		int row = 0;
		int col = 0;

		while (std::getline(in, line)) {

			char *ptr = (char *)line.c_str();
			int len = line.length();

			col = 0;

			char *start = ptr;
			for (int i = 0; i < len; i++) {

				if (ptr[i] == ',') {
					K(row, col++) = atof(start);
					start = ptr + i + 1;
				}
			}
			K(row, col) = atof(start);
			row++;
		}
		in.close();
	}
}


void LQRController::calculateCommand() {
	// calculate new command velocities
	// u is a vector of velocity commands: dX1, dY1, dX2, dY2 (1 = index, 2 = thumb)
	u = -K*state;
}


/* This function returns the calculated command to be used or printed elsewhere*/
Eigen::Vector4f LQRController::getCommand() {
	return u;
}

/* This function returns the calculated command to be used or printed elsewhere*/
Eigen::Matrix<float, 12, 1, Eigen::DontAlign> LQRController::getState() {
	return state;
}


/* This function returns the calculated command to be used or printed elsewhere*/
Eigen::Matrix<float,4,12, Eigen::DontAlign> LQRController::getGains() {
	return K;
}




/*-----------------------------------------------------------------------------*/
/* This function sets the:
	- world frame position and velocity
	- body frame velocity and acceleration
*/
void LQRController::setPos(cVector3d a_pos) {
	m_vel = linVelFilter.update((a_pos - m_pos) / deltaT, deltaT);
	getUVW();
	m_pos = a_pos;
}

/* This function sets the:
- world frame euler angles and angular velocity
- body frame angular velocity and angular acceleration
*/
void LQRController::setRot(cVector3d a_euler) {
	a_euler = yprFilter.update(a_euler, deltaT);
	m_rotvel = rotVelFilter.update((a_euler - m_euler) / deltaT, deltaT);
	getPQR();
	m_euler = a_euler;
}

/* This function updates the body frame error for use as reference input
to the LQR controller
*/
void LQRController::setError(double a_xErr, double a_zErr, double a_yawErr, double a_rollErr) {
	m_xErr = a_xErr;
	m_zErr = a_zErr;
	m_yawErr = a_yawErr;
	m_rollErr = a_rollErr;
}

/* This function calculates the body frame linear velocity and acceleration for use in the
state vector of the control system */
void LQRController::getUVW() {
	double yaw = m_euler(0); double pitch = m_euler(1); double roll = m_euler(2);
	cMatrix3d Rr(1.0, 0.0, 0.0, 0.0, cos(roll), sin(roll), 0.0, -sin(roll), cos(roll));
	cMatrix3d Rp(cos(pitch), 0.0, -sin(pitch), 0.0, 1.0, 0, sin(pitch), 0.0, cos(pitch));
	cMatrix3d Ry(cos(yaw), sin(yaw), 0.0, -sin(yaw), cos(yaw), 0.0, 0.0, 0.0, 1.0);
	cVector3d m_uvwNew = Rr*Rp*Ry*m_vel;
	m_uvwDot = (m_uvwNew - m_uvw) / deltaT;		// calculate acceleration
	m_uvw = m_uvwNew;							// update UVW
}

/* This function calculates the body frame angular velocity and acceleration for use in the
state vector of the control system */
void LQRController::getPQR() {
	double yaw = m_euler(0); double pitch = m_euler(1); double roll = m_euler(2);
	double dyaw = m_rotvel(0); double dpitch = m_rotvel(1); double droll = m_rotvel(2);

	cMatrix3d R(1.0, 0.0, sin(pitch), 0.0, cos(roll), cos(pitch)*sin(roll), 0.0, -sin(roll), cos(pitch)*cos(roll));

	cVector3d m_pqrNew = R*cVector3d(droll, dpitch, dyaw);
	m_pqrDot = (m_pqrNew - m_pqr) / deltaT;		// calculate angular acceleration
	m_pqr = m_pqrNew;							// update pqr
}

