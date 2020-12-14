#pragma once
#include "LowPassFilter.hpp"
#include "chai3d.h"

class Vec3LowPassFilter {

private:
	double filteredX;
	double filteredY;
	double filteredZ;
	LowPassFilter lpfX;
	LowPassFilter lpfY;
	LowPassFilter lpfZ;
	float m_cutOffFreq; // 10.0 / (2 * 3.14159);	// (1.5* 2 * PI) Hz

public:
	Vec3LowPassFilter();
	void setCutOffFreq(float a_cutOffFreq);
	chai3d::cVector3d update(chai3d::cVector3d, float deltaT);

};