#include "MPC.h"
using namespace std;
#define LARGE_NUM 1000.0

// resources: 
// CVXgen
// https://magiccvs.byu.edu/gitlab/lab/mqce_software_examples/blob/master/src/mpc/mpc.cpp

extern "C" {
#include "solver.h"
}

Vars vars;
Params params;
Workspace work;
Settings settings;


/* Contstructor */
MPC::MPC() {
	
	set_defaults();
	//setup_indexing();
	settings.verbose = 0;
	//settings.max_iters = 100;
	//settings.verbose_refinement = 0;

	// Initialize A Matrix (CVX is column-major)
	for (int i = 0; i < nStates; i++) {
		for (int j = 0; i < nStates; i++) {
			params.A[i + j * nStates] = 0;
		}
	}

	// Initialize B Matrix (CVX is column-major)
	for (int i = 0; i < nStates; i++) {
		for (int j = 0; i < nInputs; i++) {
			params.B[i + j * nInputs] = 0;
		}
	}

	// Initialize Q (state cost) (CVX is column-major)
	for (int i = 0; i < nStates; i++) {
		for (int j = 0; i < nStates; i++) {
			if (i == j) {
				params.Q[i + j * nStates] = 1;
				params.Q_final[i + j * nStates] = 1;
			}
			else {
				params.Q[i + j * nStates] = 0;
				params.Q_final[i + j * nStates] = 0;
			}
		}
	}

	// initialize R (input cost) (CVX is column-major)
	for (int i = 0; i < nInputs; i++) {
		for (int j = 0; i < nInputs; i++) {
			if (i == j) {
				params.R[i + j*nInputs] = 1;
			}
			else {
				params.R[i + j * nInputs] = 0;
			}
		}
	}

	// For now, just load file-stored matrices at beginning
	updateSystem();
	setConstraints();
}

/* Pass new state vector to use in calculations */
void MPC::updateAll(cVector3d a_pos, cVector3d a_euler, double a_xErr, double a_zErr, double a_yawErr, double a_rollErr, double a_X1, double a_Y1, double a_X2, double a_Y2) {

	// use controller update function to fill state vector
	LQRController::updateAll(a_pos, a_euler, a_xErr, a_zErr, a_yawErr, a_rollErr);
	
	// pass state to CVX params
	for (int i = 0; i < state.size(); i++) {
		params.x_0[i] = state[i];
	}
	//params.x_0[0] = 0.05; // for testing

	// append state with pad positions
	//for (int i = state.size(); i < nStates; i++) {
	//	params.x_0[i] = a_pantographPos[i];
	//}
	params.x_0[12] = a_X1;
	params.x_0[13] = a_Y1;
	params.x_0[14] = a_X2;
	params.x_0[15] = a_Y2;

}

/* Load A, B, Q, R matrices from file */
void MPC::updateSystem() {
	// fill each matrix from file (TO DO: clean this up later)
	vector<double> Q = readMatrixColumnMajor("../Q_mat.csv", nStates, nStates);
	for (int i = 0; i < Q.size(); i++) {
		params.Q[i] = Q[i];
		params.Q_final[i] = Q[i];
	}
	vector<double> R = readMatrixColumnMajor("../R_mat.csv", nInputs, nInputs);
	for (int i = 0; i < R.size(); i++) {
		params.R[i] = R[i];
	}
	vector<double> A = readMatrixColumnMajor("../A_mat.csv", nStates, nStates);
	for (int i = 0; i < A.size(); i++) {
		params.A[i] = A[i];
	}
	vector<double> B = readMatrixColumnMajor("../B_mat.csv", nStates, nStates);
	for (int i = 0; i < B.size(); i++) {
		params.B[i] = B[i];
	}
}

/* Set up constraints */
void MPC::setConstraints() {
	double x_max[16] = { LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, LARGE_NUM, 4.0, 4.0, 4.0, 4.0 };
	for (int i = 0; i < nStates; i++) {
		params.x_max[i] = x_max[i];
	}
}

/* Solve for optimal control output (pad velocities)*/
Eigen::Matrix<float, 4, 1, Eigen::DontAlign> MPC::getMPCinputs() {
	long num_iters = solve();
	//cout << num_iters << ", " << vars.u_0[0] << ", " << vars.u_0[1] << ", " << vars.u_0[2] << ", " << vars.u_0[3] << endl;
	if (work.converged != 1) {
		cout << "CVX Failed to converge" << endl;
		// don't change command
		//for (int i = 0; i < nInputs; i++) {
		//	command(i) = 0.0;
		//}
	}
	else {
	//	cout << "CONVERGED: ";
		for (int i = 0; i < nInputs; i++) {
			command(i) = vars.u_0[i];
		//	cout << command(i) << ", ";
		}
	}
	//cout << endl;
	return command;

}

/* This function opens the text file containing the gain matrix and stores the values in a column matrix array*/
vector<double> MPC::readMatrixColumnMajor(std::string file, int nRow, int nCol  ) {
	std::ifstream in(file);
	std::string line;

	vector<double> matrixElements(nRow*nCol, 0.0);

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
					matrixElements[row + col * (nCol)] = atof(start);
					col++;
					start = ptr + i + 1;
				}
			}
			matrixElements[row + col * (nCol)] = atof(start);
			row++;
		}
		in.close();
	}
	return matrixElements;
}
