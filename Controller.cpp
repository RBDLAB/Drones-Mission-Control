#include "Controller.h"
Controller::Controller()
{
	initPIDs();
}
void Controller::initPIDs()
{
	PIDs[0] = new PID(4, 5, 6);
	PIDs[1] = new PID(7, 8, 9);
	PIDs[2] = new PID(1, 0, 0);
	PIDs[3] = new PID(1, 0, 0);
}
void Controller::convertXYZtoAngles(float* xyz, float* to)
{

	float pitch_val = xyz[0] / sqrt(pow(xyz[1], 2) + pow(xyz[2], 2));
	float roll_val = xyz[1] / sqrt(pow(xyz[0], 2) + pow(xyz[2], 2));
	float yaw_val = FLT_MAX;


	to[0] = pitch_val;
	to[1] = roll_val;
	to[2] = yaw_val;
}
void Controller::computeControl(float* state1, float* xyz2, float* ret_value, float hdif)
{

	float xyz1[3];
	float rot1[3];

	for (int i = 0; i < 3; ++i) {
		xyz1[i] = state1[i];
	}

	for (int i = 0; i < 3; ++i) {
		rot1[i] = state1[i + 3];
	}

	float R[3][3];
	angle2dcm(rot1[2], 0, 0, R);

	float dif[3];
	matrixDiff(xyz2, xyz1, dif);

	float e_pos[3];
	matrixMult(R, dif, e_pos);


	float new_pos[3];
	toZeros(new_pos);

	for (int i = 0; i < 3; ++i) {
		new_pos[i] = PIDs[i]->compute(0, e_pos[i]);
	}
	double heading = PIDs[3]->compute(0, hdif);

	float RR[3][3];
	angle2dcm(rot1[2], rot1[1], rot1[0], RR);
	float throttle_vector_comp = 0.0;

	if (RR[2][2] > 0.1) {
		throttle_vector_comp = 1 / RR[2][2];
	}
	else {
		throttle_vector_comp = 10.0;
	}

	new_pos[2] = (new_pos[2] + GRAVITY_COMP) * throttle_vector_comp;
	

	float vals[3];
	convertXYZtoAngles(new_pos, vals);


	ret_value[0] = RPY_INIT + RPY_FACTOR * vals[0];
	ret_value[1] = RPY_INIT + RPY_FACTOR * vals[1];
	ret_value[2] = T_INIT + T_FACTOR * new_pos[2];
	ret_value[3] = RPY_INIT + RPY_FACTOR*heading;
	
	for (int i = 0; i < 4; ++i) {

		if ((i) == THROTTLE - 1) {	 

			if (ret_value[i] > T_CHANNEL_MAX) {
				ret_value[i] = T_CHANNEL_MAX;
			}
			if (ret_value[i] < T_CHANNEL_MIN) {
				ret_value[i] = T_CHANNEL_MIN;
			}

		}
		else {
			if (ret_value[i] < RPY_CHANNEL_MIN) {
				ret_value[i] = RPY_CHANNEL_MIN;
			}
			if (ret_value[i] > RPY_CHANNEL_MAX) {
				ret_value[i] = RPY_CHANNEL_MAX;
			}
		}

	}
}