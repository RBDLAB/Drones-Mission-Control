#include <ctime>
#include <iostream>
#include <chrono>
#include "timer.h"

using namespace std;

class PID {

public:

	float kp, ki, kd;
	double errSum, lastErr, lastTime;
	stoper::CTimer _time;
	bool firstTime;

	PID(float _kp, float _ki, float _kd) : kp(_kp), ki(_ki), kd(_kd), errSum(0), lastErr(0), lastTime(0)
	{
		_time.start();//now
		firstTime = true;
	}

	float compute(float input, float set_point);
	//{

	//	double time_change = 1;// 1.0 / 60 / 60 / 24;
	//	//double now = clock();//time(0)*100000.0 / 60 / 60 / 24; // convert to days

	//	//if ( lastTime != 0 ){
	//	//	time_change = (now - lastTime) / CLOCKS_PER_SEC;
	//	//}
	//	//cout << time_change << ' ';
	//	time_change = _time.getTimeAndRelease() / 1000.0;//to second
	//	double error = set_point - input;
	//	errSum+= error * time_change;
	//	double dErr = (error - lastErr) / time_change;

	//	lastErr = error;
	//	//lastTime = now;

	//	//cout << kp << ' ' << ki << ' ' << kd << ' ' << error << ' ' << errSum << ' ' << dErr << endl;
	//	return kp*error + ki*errSum + kd*dErr;
	//}

};