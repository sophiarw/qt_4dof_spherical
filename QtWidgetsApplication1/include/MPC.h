#pragma once

#include <vector>
#include "LQRController.h"



using namespace std;
class MPC : public LQRController{

public:

private:
	
	const int nInputs = 4;
	const int nStates = 8;
	Eigen::Matrix<float, 4, 1, Eigen::DontAlign> command;

public:
	MPC::MPC();

	void updateAll(cVector3d a_pos, cVector3d a_euler, double a_xErr, double a_zErr, double a_rollErr, double a_yawErr, double a_X1, double a_Y1, double a_X2, double a_Y2);
	void setConstraints();
	Eigen::Matrix<float, 4, 1, Eigen::DontAlign> getMPCinputs();
	vector<double> readMatrixColumnMajor(std::string file, int nRow, int nCol);
	void updateSystem();


};