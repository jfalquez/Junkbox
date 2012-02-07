//============================================================================
// Name        : AirGround.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description :
//============================================================================

#include <stdio.h>

#include <Eigen/Core>

#include <Mvlpp/Mvl.h>


#define PI 3.14159265


bool AmFlying( Eigen::Vector3d /* P */ ) {
	// Threshold to consider objects still on ground
	double dZThreshold = 0.10; // in meters

	// Ramp info - static info (can be inferred by MoCap)
	double dRampWidth = 1.0; 	// in meters
	double dRampHeight = 1.0; 	// in meters
	double dRampAngle = 30; 	// in deg

	// Ramp position + orientation in world reference frame (given by MoCap)
	Eigen::Matrix<double,6,1> Ramp1Pos,Ramp2Pos;
	Ramp1Pos << 5.0, 0, sin(dRampAngle*PI/180)*dRampHeight/2, 0, -dRampAngle*PI/180, 0;
	Eigen::Matrix4d Twr1 = mvl::Cart2T(Ramp1Pos);
   	Ramp2Pos << 10.0, 0, sin(dRampAngle*PI/180)*dRampHeight/2, 0, -dRampAngle*PI/180, 0;
   	Eigen::Matrix4d Twr2 = mvl::Cart2T(Ramp2Pos);

	// Point we wish to analyze; position of car (given by MoCap)
	Eigen::Vector3d P;
	Eigen::Vector4d Pt;
	P << 0.5, 0.5, 0.0;
	Pt << P, 1;
	Pt = Twr1 * Pt;
	P = Pt.block<3,1>(0,0);


	// We bring the point to each of the ramp's reference frames
	Eigen::Vector4d Pr1,Pr2;
	Pr1 << P, 1;
	Pr2 << P, 1;
	Pr1 = mvl::TInv(Twr1) * Pr1;
	Pr2 = mvl::TInv(Twr2) * Pr2;

	// Check if we are within ramp's "area"
	// < 0 should work, but octave is acting up
	if ( (fabs(Pr1(0)) - dRampHeight/2 < 0.0001) && (fabs(Pr1(1)) - dRampWidth/2 < 0.0001) && (Pr1(2) <= dZThreshold) ) {
		printf("AmFlying: On ramp 1!\n");
		return false;
	}

	if ( (fabs(Pr2(0)) - dRampHeight/2 < 0.0001) && (fabs(Pr2(1)) - dRampWidth/2 < 0.0001) && (Pr2(2) <= dZThreshold) ) {
		printf("AmFlying: On ramp 2!\n");
		return false;
	}

	// if on the floor
	if ( P(2) < 0 + dZThreshold ) {
		printf("AmFlying: On the ground.\n");
		return false;
	}

	// in the air!
	printf("AmFlying: Yup.. woohoo!\n");
	return true;
}


int main() {

	Eigen::Vector3d P;
	return AmFlying( P );
}
