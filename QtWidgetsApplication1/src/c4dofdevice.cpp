#include "c4dofdevice.h"

using namespace std;
using namespace chai3d;


c4DOFDevice::c4DOFDevice():m_th(4), m_thDes(4), m_posDes(4), m_th_init(4), centerPoint(4), neutralPos(4) {

	cout << "created device" << endl;
	m_c4DOFDeviceAvailable = true;
	m_error = false;
	m_errMessage = "";
	torsionState = true;
	shearState = true;

	// initialize kinematic variables
	m_t = 0;

	/*m_th << 0.5648, 0.5648, 0.5648, 0.5648;
	m_thDes << 0.5648, 0.5648, 0.5648, 0.5648;
	m_th_init << 0.5648, 0.5648, 0.5648, 0.5648*/
	m_th << 0, 0, 0, 0;
	m_thDes << 0, 0, 0, 0;
	m_th_init << 0, 0, 0, 0;
	m_posDes << 0, 0, -31.0, 0;

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

	if (m_location == 0) {
		//finger
		filename2 = "desired_pos_filtered_finger_forceFunction.csv";
		filename3 = "desired_pos_filtered_finger_posFunction.csv";
	}
	else {
		//case for thumb
		filename2 = "desired_pos_filtered_thumb_forceFunction.csv";
		filename3 = "desired_pos_filtered_thumb_posFunction.csv";
	}

	file.open(filename);
	file << "x_pos, y_pos, z_pos, theta_pos, joint1, joint2, joint3, joint4, motor1, motor2, motor3, motor4" << endl;

	file2.open(filename2);
	file2 << "x_pos, y_pos, z_pos, theta_pos" << endl;

	file3.open(filename3);
	file3 << "x_pos, y_pos, z_pos, theta_pos" << endl;

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

	Eigen::Vector4d X;
	Eigen::Vector4d Y;
	Eigen::Vector4d Z;

	//original DB vectors
	Eigen::Vector3d DBc1;
	Eigen::Vector3d DBc2;
	Eigen::Vector3d DBc3;
	Eigen::Vector3d DBc4;

	//rotated DB vectors
	Eigen::Vector3d DBs1;
	Eigen::Vector3d DBs2;
	Eigen::Vector3d DBs3;
	Eigen::Vector3d DBs4;

	DBc1 << d / 2 + d1 - 0.5 * hi * sin(theta_d),
		h1 + 0.5 * hi * cos(theta_d),
		0;

	DBc2 << - d / 2 - d1 - 0.5 * hi * sin(theta_d),
		h1 + 0.5 * hi * cos(theta_d),
		0;

	DBc3 << - d / 2 - d1 + 0.5 * hi * sin(theta_d),
		- h1 - 0.5 * hi * cos(theta_d),
		0;

	DBc4 << d / 2 + d1 + 0.5 * hi * sin(theta_d),
		- h1 - 0.5 * hi * cos(theta_d),
		0;

	Eigen::Vector3d n1(0, 0, -1);
	Eigen::Vector3d n2(2*x_d, 2*y_d, 2*z_d);

	Eigen::Vector3d axis;
	axis = n2.cross(n1);
	axis = axis * 1 / axis.norm();

	double angle = acos(n1.dot(n2) / n1.norm() / n2.norm());

	//rotate DB vectors
	if ((x_d == 0) && (y_d == 0)) {
		DBs1 = Eigen::Vector3d::Zero() + DBc1;
		DBs2 = Eigen::Vector3d::Zero() + DBc2;
		DBs3 = Eigen::Vector3d::Zero() + DBc3;
		DBs4 = Eigen::Vector3d::Zero() + DBc4;

	}
	else {
		DBs1 = cos(angle)*DBc1 + sin(angle) * DBc1.cross(axis) + (1 - cos(angle)) * axis.dot(DBc1) * axis;
		DBs2 = cos(angle)*DBc2 + sin(angle) * DBc2.cross(axis) + (1 - cos(angle)) * axis.dot(DBc2) * axis;
		DBs3 = cos(angle)*DBc3 + sin(angle) * DBc3.cross(axis) + (1 - cos(angle)) * axis.dot(DBc3) * axis;
		DBs4 = cos(angle)*DBc4 + sin(angle) * DBc4.cross(axis) + (1 - cos(angle)) * axis.dot(DBc4) * axis;
	}
	

	X <<  x_d - xa + DBs1[0],
		  x_d + xa + DBs2[0],
		  x_d + xa + DBs3[0],
		  x_d - xa + DBs4[0];

	Y << y_d - ya + DBs1[1],
		 y_d - ya + DBs2[1],
		 y_d + ya + DBs3[1],
		 y_d + ya + DBs4[1];

	Z << z_d - za + DBs1[2],
		z_d - za + DBs2[2],
		z_d - za + DBs3[2],
		z_d - za + DBs4[2];

	Eigen::Vector4d I;
	Eigen::Vector4d J;
	Eigen::Vector4d K;
	Eigen::Vector4d Delta;

	// Joint 1
	I[0] = 2 * Z[0] * Li;
	J[0] = -sqrt(2) * X[0] * Li - sqrt(2) * Y[0] * Li;
	K[0] = pow(Li, 2) - pow(li, 2) + pow(X[0], 2) + pow(Y[0], 2) + pow(Z[0], 2);
	Delta[0] = pow(I[0], 2) - pow(K[0], 2) + pow(J[0], 2);

	// Joint 2
	I[1] = 2 * Z[1] * Li;
	J[1] = sqrt(2) * X[1] * Li - sqrt(2) * Y[1] * Li;
	K[1] = pow(Li, 2) - pow(li, 2) + pow(X[1], 2) + pow(Y[1], 2) + pow(Z[1], 2);
	Delta[1] = pow(I[1], 2) - pow(K[1], 2) + pow(J[1], 2);

	// Joint 3
	I[2] = 2 * Z[2] * Li;
	J[2] = sqrt(2) * X[2] * Li + sqrt(2) * Y[2] * Li;
	K[2] = pow(Li, 2) - pow(li, 2) + pow(X[2], 2) + pow(Y[2], 2) + pow(Z[2], 2);
	Delta[2] = pow(I[2], 2) - pow(K[2], 2) + pow(J[2], 2);

	//Joint 4
	I[3] = 2 * Z[3] * Li;
	J[3] = -sqrt(2) * X[3] * Li + sqrt(2) * Y[3] * Li;
	K[3] = pow(Li, 2) - pow(li, 2) + pow(X[3], 2) + pow(Y[3], 2) + pow(Z[3], 2);
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

	file << x_d << ", " << y_d << ", " << z_d << ", " << theta_d  << ", ";
	file << jointAngles.x() << ", " << jointAngles.y() << ", " << jointAngles.z() << ", " << jointAngles.w() << ", ";
	file << motorAngles.x() << ", " << motorAngles.y() << ", " << motorAngles.z() << ", " << motorAngles.w() << endl;

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

	file3 << desiredPos.x() << ", " << desiredPos.y() << ", " << desiredPos.z() << ", " << desiredPos.w() << endl;

	inverseKinematics();


}


void c4DOFDevice::setForce(const Eigen::Ref<Eigen::Vector4d> a_force) {

	Eigen::Vector4d pos(a_force.x() / k_skin_shear, a_force.y() / k_skin_shear, a_force.z() / k_skin_normal, a_force[3] / k_skin_rotational);
	//Eigen::Vector4d pos(0, 0, 0, 0);
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

	if (torsionState == false) {
		desiredPos[3] = 0;
	}

	if (shearState == false) {
		desiredPos[0] = 0;
		desiredPos[1] = 0;
	}

	float deltaT = filterTimer.getCurrentTimeSeconds();
	desiredPos[0] = lpfX.update(desiredPos.x(), deltaT);
	desiredPos[1] = lpfY.update(desiredPos.y(), deltaT);
	desiredPos[2] = lpfZ.update(desiredPos.z(), deltaT);
	desiredPos[3] = lpftheta.update(desiredPos.w(), deltaT);
	filterTimer.reset();
	
	m_posDes = desiredPos;

	file2 << desiredPos.x() << ", " << desiredPos.y() << ", " << desiredPos.z() << ", " << desiredPos.w() << endl;

	inverseKinematics();
}





void c4DOFDevice::setNeutralPos(const double x, const double y, const double z, const double theta) {
	neutralPos[0] = x;
	neutralPos[1] = y;
	neutralPos[2] = z;
	neutralPos[3] = theta;

	Eigen::Vector4d newPos(0, 0, 0, 0);
	setPos(newPos);
}

void c4DOFDevice::setTorsionState(const bool state) {
	torsionState = state;
	//file << "torsion state: " << torsionState << endl;
}

void c4DOFDevice::setShearState(const bool state) {
	shearState = state;
	//file << "torsion state: " << torsionState << endl;
}


