#include "c4dofdevice.h"

using namespace std;
using namespace chai3d;

//:m_th(0.0, 0.0, 0.0, 0.0), m_thDes(0.0, 0.0, 0.0, 0.0), m_posDes(0.0, 0.0, 0.0, 0.0), joints_thDes(0.0, 0.0, 0.0, 0.0), centerPoint(0.0, 0.0, 0.0, 0.0), neutralPos(0.0, 0.0, 0.0, 0.0)
c4DOFDevice::c4DOFDevice():m_th(4), m_thDes(4), m_posDes(4), joints_thDes(4), centerPoint(4), neutralPos(4) {
		// device available for connection

	cout << "created device" << endl;
	m_c4DOFDeviceAvailable = true;
	m_error = false;
	m_errMessage = "";

	lpfX.setCutOffFrequency(cutOffFreqFinger);
	lpfZ.setCutOffFrequency(cutOffFreqFinger);

	// initialize kinematic variables
	m_t = 0;

	m_th << 0, 0, 0, 0; //need to figure out what this should be
	m_thDes << 0, 0, 0, 0;
	m_posDes << 0, 0, 0, 0;
	joints_thDes << 0, 0, 0, 0;
	centerPoint << 0, 0, 0, 0;  //[mm]				// pantograph x positive to left when looking at motor axle, z positive down
	neutralPos << 0.0, 0.0, -20.0, 0.0; // There is a 20mm offset on the z position

	//assign the differences between the thumb and finger using

	if (m_location == 0) {
		//case for index finger
	}
	else {
		//case for thumb
	}

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
			if ((~(isnan(Q1))) && (Q1 < PI / 2) && (Q1 > 0)) {
				if ((~(isnan(Q2))) && (Q2 < PI / 2) && (Q2 > 0)) {
					jointAngles[i] = min(Q1, Q2);
				}
				else {
					jointAngles[i] = Q1;
				}
			}
			else if ((~(isnan(Q2))) && (Q2 < PI / 2) && (Q2 > 0)) {
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
		joints_thDes << desiredAngles;
		m_thDes << motorAngles;
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


void c4DOFDevice::setPos(const Eigen::Ref<Eigen::Vector4d> a_force) {

	double xRange = 7.0; // simulated max is 7.8  [mm]
	double yRange = 9.0; // simulated max is 10  [mm]
	double zRange = 4.5; //simulated is 5 [mm]
	double thetaRange = PI / 6;


	Eigen::Vector4d pos(a_force.x() / k_skin_shear, a_force.y()/k_skin_shear, -a_force.z() / k_skin_normal, a_force[3]/k_skin_rotational);

	//limit workspace motion 
	Eigen::Vector4d desiredPos = pos + neutralPos;

	double xPosLimit = neutralPos[0] + xRange;
	double xNegLimit = neutralPos[0] - xRange;
	double yPosLimit = neutralPos[1] + yRange;
	double yNegLimit = neutralPos[1] - yRange;
	double zPosLimit = neutralPos[2] + zRange;
	double zNegLimit = neutralPos[2] - zRange;
	double thetaPosLimit = neutralPos[3] + thetaRange;
	double thetaNegLimit = neutralPos[3] - thetaRange;

	if (desiredPos[0] > xPosLimit) desiredPos[0] = xPosLimit;
	if (desiredPos[0] < xNegLimit) desiredPos[0] = xNegLimit;
	if (desiredPos[1] > yPosLimit) desiredPos[1] = yPosLimit;
	if (desiredPos[1] < yNegLimit) desiredPos[1] = yNegLimit;
	if (desiredPos[2] > zPosLimit) desiredPos[2] = zPosLimit;
	if (desiredPos[2] < zNegLimit) desiredPos[2] = zNegLimit;
	if (desiredPos[3] > thetaPosLimit) desiredPos[3] = thetaPosLimit;
	if (desiredPos[3] < thetaNegLimit) desiredPos[3] = thetaNegLimit;
	
	m_posDes = desiredPos;

	//cout << desiredPos.x() << " ," << desiredPos.y() << " ," << desiredPos.z()  << " ," << desiredPos[3] << endl;

	inverseKinematics();


}


void c4DOFDevice::plot_vectors() {
	double x1 = xa;
	double y1 = ya;
	double z1 = za;

	double q1 = joints_thDes[0];
	double q2 = joints_thDes[1]; 
	double q3 = joints_thDes[2];
	double q4 = joints_thDes[3];

	double x = m_posDes[0];
	double y = m_posDes[1];
	double z = m_posDes[2];
	double theta = m_posDes[3];

	Eigen::Vector3d P1(x1, y1, z1); //Position of 1st actuator
	Eigen::Vector3d PA1(cos(PI / 4)*cos(q1)*Li, sin(PI / 4)*cos(q1)*Li, -sin(q1)*Li);
	Eigen::Vector3d A1;
	A1 << P1 + Eigen::Vector3d(cos(PI / 4)*cos(q1)*Li, sin(PI / 4)*cos(q1)*Li, -sin(q1)*Li);
	Eigen::Vector3d B1;
	B1 << (x - .5*hi*sin(theta) + d1 + d / 2),  (y + .5*hi*cos(theta) + h1),  z;
	
	Eigen::Vector3d AB1= B1 - A1;

	// Arm2
	double x2 = -xa;
	double y2 = ya;
	double z2 = za;

	Eigen::Vector3d P2(x2, y2, z2); //Position of 2nd actuator
	Eigen::Vector3d	PA2(cos(3 * PI / 4)*cos(q2)*Li, sin(3 * PI / 4)*cos(q2)*Li, -sin(q2)*Li);
	Eigen::Vector3d A2;
	A2 << P2 + Eigen::Vector3d((cos(3 * PI / 4)*cos(q2)*Li), (sin(3 *PI / 4)*cos(q2)*Li), -sin(q2)*Li);
	Eigen::Vector3d B2;
	B2 << (x - 0.5*hi*sin(theta) - d1 - d / 2),  (y + 0.5*hi*cos(theta) + h1),  z;
	Eigen::Vector3d AB2 = B2 - A2;

	// Arm3
	double x3 = -xa;
	double y3 = -ya;
	double z3 = za;

	Eigen::Vector3d P3(x3, y3, z3); //Position of 3rd actuator
	Eigen::Vector3d PA3(cos(5 * PI / 4)*cos(q3)*Li, sin(5 * PI / 4)*cos(q3)*Li, -sin(q3)*Li);
	Eigen::Vector3d A3;
	A3 << P3 + Eigen::Vector3d(cos(5 * PI / 4)*cos(q3)*Li, sin(5 * PI / 4)*cos(q3)*Li, -sin(q3)*Li);
	Eigen::Vector3d B3((x - d1 - d / 2 + .5*hi*sin(theta)),  (y - h1 - .5*hi*cos(theta)), z);
	Eigen::Vector3d AB3 = B3 - A3;

	// Arm4
	double x4 = xa;
	double y4 = -ya;
	double z4 = za;

	Eigen::Vector3d P4(x4, y4, z4); //Position of 4th actuator
	Eigen::Vector3d PA4(cos(7 * PI / 4)*cos(q4)*Li, sin(7 * PI / 4)*cos(q4)*Li, -sin(q4)*Li);
	Eigen::Vector3d A4;
	A4 << P4 + Eigen::Vector3d(cos(7 * PI / 4)*cos(q4)*Li, sin(7 * PI / 4)*cos(q4)*Li, -sin(q4)*Li);
	Eigen::Vector3d B4(x + d1 + d / 2 + .5*hi*sin(theta), y - h1 - .5*hi*cos(theta), z);
	Eigen::Vector3d AB4 = B4 - A4;

	link1_1 = { 0, 0, za };
	link1_2 = { P1[1], P1[2], P1[3] };
	link1_3 = { A1[1], A1[2], A1[3] };
	link1_4 = { B1[1], B1[2], B1[3] };
	link1_5 = { B1[1]-d1, B1[2], B1[3] };
	link1_6 = { B1[1]-d1, B1[2] - h1, B1[3] };
	
	link2_1 = { 0, 0, za };
	link2_2 = { P2[1], P2[2], P2[3] };
	link2_3 = { A2[1], A2[2], A2[3] };
	link2_4 = { B2[1], B2[2], B2[3] };
	link2_5 = { B2[1] + d1, B2[2], B2[3] };
	link2_6 = { B2[1] + d1, B2[2] - h1, B2[3] };

	link3_1 = { 0, 0, za };
	link3_2 = { P3[1], P3[2], P3[3] };
	link3_3 = { A3[1], A3[2], A3[3] };
	link3_4 = { B3[1], B3[2], B3[3] };
	link3_5 = { B3[1] + d1, B3[2], B3[3] };
	link3_6 = { B3[1] + d1, B3[2] + h1, B3[3] };

	link4_1 = { 0, 0, za };
	link4_2 = { P4[1], P4[2], P4[3] };
	link4_3 = { A4[1], A4[2], A4[3] };
	link4_4 = { B4[1], B4[2], B4[3] };
	link4_5 = { B4[1] - d1, B4[2], B4[3] };
	link4_6 = { B4[1] - d1, B4[2] + h1, B4[3] };


}

void c4DOFDevice::setForce(const Eigen::Ref<Eigen::Vector4d> a_force) {

	double xRange = 5.0;
	double yRange = 5.0;
	double zRange = 4.5;
	double thetaRange = PI / 6;
	//limit workspace motion 

	//calculate displacements based on desired forces
	Eigen::Vector4d desiredPos(a_force[0] / k_skin_shear, a_force[1] / k_skin_normal, a_force[2] / k_skin_shear, a_force[3] / k_skin_rotational);


	//check if values are in range if not set them to maximum values
	double xPosLimit = this->neutralPos[0] + xRange;
	double xNegLimit = this->neutralPos[0] - xRange;
	double yPosLimit = this->neutralPos[1] + yRange;
	double yNegLimit = this->neutralPos[1] - yRange;
	double zPosLimit = this->neutralPos[2] + zRange;
	double zNegLimit = this->neutralPos[2] - zRange;
	double thetaPosLimit = this->neutralPos[3] + thetaRange;
	double thetaNegLimit = this->neutralPos[3] - thetaRange;

	if (desiredPos[0] > xPosLimit) desiredPos[0] = xPosLimit;
	if (desiredPos[0] < xNegLimit) desiredPos[0] = xNegLimit;
	if (desiredPos[1] > yPosLimit) desiredPos[1] = yPosLimit;
	if (desiredPos[1] < yNegLimit) desiredPos[1] = yNegLimit;
	if (desiredPos[2] > zPosLimit) desiredPos[2] = zPosLimit;
	if (desiredPos[2] < zNegLimit) desiredPos[2] = zNegLimit;
	if (desiredPos[3] > thetaPosLimit) desiredPos[3] = thetaPosLimit;
	if (desiredPos[3] < thetaNegLimit) desiredPos[3] = thetaNegLimit;


	float deltaT = c4DOFDeviceTimer.getCurrentTimeSeconds();
	c4DOFDeviceTimer.reset();

	// NOTE: What Julie had, is it necessary?
	//m_posDes.x(lpfX.update(centerPoint.x() + stretch.x(), deltaT));
	//m_posDes.z(lpfZ.update(centerPoint.z() + stretch.z(), deltaT));
	m_posDes = desiredPos;
	inverseKinematics();
}





void c4DOFDevice::setNeutralPos(const double x, const double y, const double z, const double theta) {
	//qDebug() << "setting Neutral Pos" << endl;
	neutralPos[0] = x;
	neutralPos[1] = y;
	neutralPos[2] = z;
	neutralPos[3] = theta;
	//qDebug() << "The new neutral position is" << neutralPos[0] << ", " << neutralPos[1] << ", " << neutralPos[2] << ", " << neutralPos[3] << endl;
}


