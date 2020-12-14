#include "vec3LPF.h"

Vec3LowPassFilter::Vec3LowPassFilter() {
	filteredX = 0.0;
	filteredY = 0.0;
	filteredZ = 0.0;
	m_cutOffFreq = -1.0;
}


void Vec3LowPassFilter::setCutOffFreq(float a_cutOffFreq) {
	// update cutoff frequency;
	m_cutOffFreq = a_cutOffFreq;
	lpfX.setCutOffFrequency(a_cutOffFreq);
	lpfY.setCutOffFrequency(a_cutOffFreq);
	lpfZ.setCutOffFrequency(a_cutOffFreq);
}

chai3d::cVector3d Vec3LowPassFilter::update(chai3d::cVector3d newVec, float deltaT) {
	if (m_cutOffFreq > 0.0) {
		filteredX = lpfX.update(newVec(0), deltaT);
		filteredY = lpfY.update(newVec(1), deltaT);
		filteredZ = lpfZ.update(newVec(2), deltaT);

		return chai3d::cVector3d(filteredX, filteredY, filteredZ);
	}
	else {
		return chai3d::cVector3d(0.0, 0.0, 0.0);
	}
}