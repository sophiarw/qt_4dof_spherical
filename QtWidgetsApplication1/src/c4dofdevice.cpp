#include "c4dofdevice.h"

using namespace std;
using namespace chai3d;


c4DOFDevice::c4DOFDevice():m_th(4), m_thDes(4), m_posDes(4), m_th_init(4), centerPoint(4), neutralPos(4) {

	cout << "created device" << endl;
	m_c4DOFDeviceAvailable = true;
	m_error = false;
	m_errMessage = "";

	// initialize kinematic variables
	m_t = 0;

	/*m_th << 0.5648, 0.5648, 0.5648, 0.5648;
	m_thDes << 0.5648, 0.5648, 0.5648, 0.5648;
	m_th_init << 0.5648, 0.5648, 0.5648, 0.5648*/
	m_th << 0, 0, 0, 0;
	m_thDes << 0, 0, 0, 0;
	m_th_init << 0, 0, 0, 0;
	m_posDes << 0, 0, -31, 0;

	inverseKinematics(); //sets m_thDes with m_th_init equal to zeros

						 //set m_th and m_th_init to that initial location
	m_th << m_thDes.x(), m_thDes.y(), m_thDes.z(), m_thDes.w();
	m_th_init << m_thDes.x(), m_thDes.y(), m_thDes.z(), m_thDes.w();

	//recalculate m_thDes give the updated m_th_init
	inverseKinematics();


	centerPoint << 0, 0, 0, 0;  //[mm]				// pantograph x positive to left when looking at motor axle, z positive down
	neutralPos << m_posDes.x(), m_posDes.y(), m_posDes.z(), m_posDes.w(); // There is a 20mm offset on the z position

	//filter parameters
	lpfX.setCutOffFrequency(cutOffFreqFinger);
	lpfY.setCutOffFrequency(cutOffFreqFinger);
	lpfZ.setCutOffFrequency(cutOffFreqFinger);
	lpftheta.setCutOffFrequency(cutOffFreqFinger);
	filterTimer.start();

	


	//assign the differences between the thumb and finger using

	if (m_location == 0) {
		//finger
		filename = "desired_pos_filtered_finger.csv";
	}
	else {
		//case for thumb
		filename = "desired_pos_filtered_thumb.csv";
	}

	file.open(filename);
	file << "x_pos, y_pos, z_pos, theta_pos" << endl;

	link1_1 = { 0, 0, 0 };
	link1_2 = { 0, 0, 0 };
	link1_3 = { 0, 0, 0 };
	link1_4 = { 0, 0, 0 };
	link1_5 = { 0, 0, 0 };
	link1_6 = { 0, 0, 0 };

	link2_1 = { 0, 0, 0 };
	link2_2 = { 0, 0, 0 };
	link2_3 = { 0, 0, 0 };
	link2_4 = { 0, 0, 0 };
	link2_5 = { 0, 0, 0 };
	link2_6 = { 0, 0, 0 };

	link3_1 = { 0, 0, 0 };
	link3_2 = { 0, 0, 0 };
	link3_3 = { 0, 0, 0 };
	link3_4 = { 0, 0, 0 };
	link3_5 = { 0, 0, 0 };
	link3_6 = { 0, 0, 0 };

	link4_1 = { 0, 0, 0 };
	link4_2 = { 0, 0, 0 };
	link4_3 = { 0, 0, 0 };
	link4_4 = { 0, 0, 0 };
	link4_5 = { 0, 0, 0 };
	link4_6 = { 0, 0, 0 };

	c4DOFDeviceTimer.start();
	
	m_c4DOFDeviceReady = true;


}

c4DOFDevice::~c4DOFDevice()
{
}



void c4DOFDevice::inverseKinematics(){
	Eigen::Vector4d motorAngles;
	Eigen::Vector4d jointAngles;
	Eigen::Vector4d desiredAngles;
	double x_d = m_posDes[0]; double y_d = m_posDes[1]; double z_d = m_posDes[2]; double theta_d =  m_posDes[3];

	//cout << "Desired position:" << x_d << ", " << y_d << ", " << z_d << ", " << theta_d << endl;

	double Z = z_d - za;
	Eigen::Vector4d X;
	Eigen::Vector4d Y;

	X << d / 2 + d1 + x_d - xa - 0.5 * hi * sin(theta_d),
		d / 2 + d1 - x_d - xa + 0.5 * hi * sin(theta_d),
		x_d - d1 - d / 2 + xa + 0.5 * hi * sin(theta_d),
		d / 2 + d1 + x_d - xa + 0.5 * hi * sin(theta_d);

	Y << h1 + y_d - ya + 0.5 * hi * cos(theta_d),
		h1 + y_d - ya + 0.5 * hi * cos(theta_d),
		y_d - h1 + ya - 0.5 * hi * cos(theta_d),
		y_d - h1 + ya - 0.5 * hi * cos(theta_d);

	Eigen::Vector4d I;
	Eigen::Vector4d J;
	Eigen::Vector4d K;
	Eigen::Vector4d Delta;

	// Joint 1
	I[0] = 2 * Z * Li;
	J[0] = -sqrt(2) * X[0] * Li - sqrt(2) * Y[0] * Li;
	K[0] = pow(Li, 2) - pow(li, 2) + pow(X[0], 2) + pow(Y[0], 2) + pow(Z, 2);
	Delta[0] = pow(I[0], 2) - pow(K[0], 2) + pow(J[0], 2);

	// Joint 2
	I[1] = 2 * Z * Li;
	J[1] = -sqrt(2) * X[1] * Li - sqrt(2) * Y[1] * Li;
	K[1] = pow(Li, 2) - pow(li, 2) + pow(X[1], 2) + pow(Y[1], 2) + pow(Z, 2);
	Delta[1] = pow(I[1], 2) - pow(K[1], 2) + pow(J[1], 2);

	// Joint 3
	I[2] = 2 * Z * Li;
	J[2] = sqrt(2) * X[2] * Li + sqrt(2) * Y[2] * Li;
	K[2] = pow(Li, 2) - pow(li, 2) + pow(X[2], 2) + pow(Y[2], 2) + pow(Z, 2);
	Delta[2] = pow(I[2], 2) - pow(K[2], 2) + pow(J[2], 2);

	//Joint 4
	I[3] = 2 * Z * Li;
	J[3] = -sqrt(2) * X[3] * Li + sqrt(2) * Y[3] * Li;
	K[3] = pow(Li, 2) - pow(li, 2) + pow(X[3], 2) + pow(Y[3], 2) + pow(Z, 2);
	Delta[3] = pow(I[3], 2) - pow(K[3], 2) + pow(J[3], 2);

	bool no_nan = true; //boolean to keep track whether a specific joint angle was not reachable
	for (int i = 0; i < 4; i++) {
		double Q1 = 2 * atan((-I[i] + sqrt(Delta[i])) / (K[i] - J[i]));
		double Q2 = 2 * atan((-I[i] - sqrt(Delta[i])) / (K[i] - J[i]));
		if (isnan(Q1) && isnan(Q2)) {
			//not reachable
			no_nan = false;
		}
		else {
			if (((!(isnan(Q1))) && (Q1 < PI / 2)) && (Q1 > 0)) {
				if (((!(isnan(Q2))) && (Q2 < PI / 2)) && (Q2 > 0)) {
					jointAngles[i] = min(Q1, Q2);
				}
				else {
					jointAngles[i] = Q1;
				}
			}
			else if (((!(isnan(Q2))) && (Q2 < PI / 2)) && (Q2 > 0)) {
				jointAngles[i] = Q2;
				
			}
			else {
				//not reachable
				no_nan = false;
			}
		}


		//setting joint limits
		if (jointAngles[i] > (80 * PI / 180)) {
			desiredAngles[i] = (80 * PI / 180);
		}
		else if (jointAngles[i] < (25 * PI / 180)) {
			desiredAngles[i] = (25 * PI / 180);
		}
		else {
			desiredAngles[i] = jointAngles[i];
		}


		//get motor angles
		motorAngles[i] = fourbar(desiredAngles[i]);
		
	
	}

	if (no_nan) { //if there were no nans, then set the output variables
		m_thDes << -(motorAngles + m_th_init);
	}

	//DEBUG
	if (0){
		cout << "Motor Angles:" << motorAngles.x() << " " << motorAngles.y() << " " << motorAngles.z() << " " << motorAngles.w() << endl;
		cout << "Joint Angles:" << jointAngles.x() << " " << jointAngles.y() << " " << jointAngles.z() << " " << jointAngles.w() << endl;

		cout << "Desired Pos:" << m_posDes.x() << " " << m_posDes.y() << " " << m_posDes.z() << " " << m_posDes.w() << endl;

	}


}

double c4DOFDevice::fourbar(double angle) {
	double theta = angle + ((-21.1 + angle_offset1) * PI / 180);
	if (0) {
		cout << "theta: " << theta << endl;
	}
	double A1 = pow(af, 2) - pow(bf, 2) + pow(cf, 2) + pow(df, 2) - 2 * af * df * cos(theta);
	double A2 = 2 * cf * df - 2 * af * cf * cos(theta);
	double A3 = -2 * af * cf * sin(theta);
	double theta_1 = 2 * atan((-A3 + sqrt(pow(A3, 2) - pow(A1, 2) + pow(A2, 2))) / (A1 - A2));
	double theta_2 = 2 * atan((-A3 - sqrt(pow(A3, 2) - pow(A1, 2) + pow(A2, 2))) / (A1 - A2));
	if (theta_1 > (-25 * PI / 180) && theta_1 < (95 * PI / 180)) {
		return theta_1 + (21.1 * PI / 180);
	}
	else {
		return theta_2 + (21.1 * PI / 180);
	}


}


void c4DOFDevice::setPos(const Eigen::Ref<Eigen::Vector4d> pos) {

	//take into account the neutralPos
	Eigen::Vector4d desiredPos = pos + neutralPos;

	


	//limit workspace motion
	double xPosLimit =  xRange;
	double xNegLimit = - xRange;
	double yPosLimit =  yRange;
	double yNegLimit = - yRange;
	double zPosLimit = zRange_max;
	double zNegLimit = zRange_min;
	double thetaPosLimit = thetaRange;
	double thetaNegLimit = - thetaRange;

	if (desiredPos[0] > xPosLimit) desiredPos[0] = xPosLimit;
	if (desiredPos[0] < xNegLimit) desiredPos[0] = xNegLimit;
	if (desiredPos[1] > yPosLimit) desiredPos[1] = yPosLimit;
	if (desiredPos[1] < yNegLimit) desiredPos[1] = yNegLimit;
	if (desiredPos[2] > zPosLimit) desiredPos[2] = zPosLimit;
	if (desiredPos[2] < zNegLimit) desiredPos[2] = zNegLimit;
	if (desiredPos[3] > thetaPosLimit) desiredPos[3] = thetaPosLimit;
	if (desiredPos[3] < thetaNegLimit) desiredPos[3] = thetaNegLimit;

	float deltaT = filterTimer.getCurrentTimeSeconds();
	desiredPos[0] = lpfX.update(desiredPos.x(), deltaT);
	desiredPos[1] = lpfY.update(desiredPos.y(), deltaT);
	desiredPos[2] = lpfZ.update(desiredPos.z(), deltaT);
	desiredPos[3] = lpftheta.update(desiredPos.w(), deltaT);
	filterTimer.reset();
	
	m_posDes = desiredPos;

	file << desiredPos.x() << ", " << desiredPos.y() << ", " << desiredPos.z() << ", " << desiredPos.w() << endl;

	inverseKinematics();


}


void c4DOFDevice::setForce(const Eigen::Ref<Eigen::Vector4d> a_force) {

	Eigen::Vector4d pos(a_force.x() / k_skin_shear, a_force.y() / k_skin_shear, a_force.z() / k_skin_normal, a_force[3] / k_skin_rotational);

	//take into account the neutralPos
	Eigen::Vector4d desiredPos = pos + neutralPos;


	//limit workspace motion
	double xPosLimit = xRange;
	double xNegLimit = -xRange;
	double yPosLimit = yRange;
	double yNegLimit = -yRange;
	double zPosLimit = zRange_max;
	double zNegLimit = zRange_min;
	double thetaPosLimit = thetaRange;
	double thetaNegLimit = -thetaRange;

	if (desiredPos[0] > xPosLimit) desiredPos[0] = xPosLimit;
	if (desiredPos[0] < xNegLimit) desiredPos[0] = xNegLimit;
	if (desiredPos[1] > yPosLimit) desiredPos[1] = yPosLimit;
	if (desiredPos[1] < yNegLimit) desiredPos[1] = yNegLimit;
	if (desiredPos[2] > zPosLimit) desiredPos[2] = zPosLimit;
	if (desiredPos[2] < zNegLimit) desiredPos[2] = zNegLimit;
	if (desiredPos[3] > thetaPosLimit) desiredPos[3] = thetaPosLimit;
	if (desiredPos[3] < thetaNegLimit) desiredPos[3] = thetaNegLimit;

	//desiredPos[3] = 0;
	m_posDes = desiredPos;

	float deltaT = filterTimer.getCurrentTimeSeconds();
	desiredPos[0] = lpfX.update(desiredPos.x(), deltaT);
	desiredPos[1] = lpfY.update(desiredPos.y(), deltaT);
	desiredPos[2] = lpfZ.update(desiredPos.z(), deltaT);
	desiredPos[3] = lpftheta.update(desiredPos.w(), deltaT);
	filterTimer.reset();
	

	file << desiredPos.x() << ", " << desiredPos.y() << ", " << desiredPos.z() << ", " << desiredPos.w() << endl;

	inverseKinematics();
}





void c4DOFDevice::setNeutralPos(const double x, const double y, const double z, const double theta) {
	//qDebug() << "setting Neutral Pos" << endl;
	neutralPos[0] = x;
	neutralPos[1] = y;
	neutralPos[2] = z;
	neutralPos[3] = theta;

	Eigen::Vector4d newPos(0, 0, 0, 0);
	setPos(newPos);
	//qDebug() << "The new neutral position is" << neutralPos[0] << ", " << neutralPos[1] << ", " << neutralPos[2] << ", " << neutralPos[3] << endl;
}


