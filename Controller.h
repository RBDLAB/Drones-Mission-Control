#pragma once
#include "PID.h"
#include <math.h>
#include <limits>
#include <cfloat>

#define PITCH		1
#define ROLL		2
#define THROTTLE	3
#define YAW			4
#define EPSILON		0.0001
#define T_CHANNEL_MIN	1100
#define T_CHANNEL_MAX	1800
#define RPY_CHANNEL_MIN	1250
#define RPY_CHANNEL_MAX	1750
#define T_INIT			1000
#define T_FACTOR		1000
#define	RPY_INIT		1500
#define	RPY_FACTOR		500
#define GRAVITY_COMP	0.4	

using namespace std;

class Controller {

public:
	PID* PIDs[4];

	Controller();

	void initPIDs();

	void convertXYZtoAngles(float* xyz, float* to);
	void matrixDiff(float* m1, float* m2, float* m) {

		for (int i = 0; i < 3; ++i) {
			m[i] = m1[i] - m2[i];
		}
	}

	void toZeros(float* m) {
		for (int i = 0; i < 3; ++i) {
			m[i] = 0.0;
		}
	}

	void matrixMult(float m1[][3], float m2[], float* res) {

		for (int i = 0; i < 3; ++i) {
			res[i] = 0;
		}

		
		for (int i = 0; i < 3; ++i) { 
			for (int j = 0; j < 3; ++j) { 
				res[i] += m1[i][j] * m2[j];
			}
		}
	}

	/*
	*
	*  THIS FUNCTION DOES NOTHING!!
	*
	*/
	void angle2dcm(float x, float y, float z, float ret[][3]) {

		float angles[3] = { x , y , z };
		float cang[3];
		float sang[3];

		for (int i = 0; i < 3; ++i) {
			cang[i] = cos(angles[i]);
			sang[i] = sin(angles[i]);
		}

		ret[0][0] = cang[1] * cang[0];
		ret[0][1] = cang[1] * sang[0];
		ret[0][2] = -sang[1];

		ret[1][0] = sang[2] * sang[1] * cang[0] - cang[2] * sang[0];
		ret[1][1] = sang[2] * sang[1] * sang[0] + cang[2] * cang[0];
		ret[1][2] = sang[2] * cang[1];

		ret[2][0] = cang[2] * sang[1] * cang[0] + sang[2] * sang[0];
		ret[2][1] = cang[2] * sang[1] * sang[0] - sang[2] * cang[0];
		ret[2][2] = cang[2] * cang[1];
	}

	void computeControl(float* state1, float* xyz2, float* ret_value, float hdif = 0);
	
};