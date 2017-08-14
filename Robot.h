#pragma once

#include "QuadSerial.h"
#include "Utilities.h"
#include "Controller.h"
#include "timer.h"
#include "ThreadLock.h"
#include "Mission.h"


class Robot
{
public:
	Robot();
	~Robot();
	void UpdateMission(float* pos, float* qrot, long globalFC);
	void OpenSerial(int portName);
	void SetPid(int xyz, double p, double i, double d);
	void SetHeight(double meters) { height = meters; }
	void SetTarget(double x, double y, double z);
	bool IsEnable() { return callbackCount > 0; }
	void GetOrigen(float& x, float& y, float& z);
	void ShutDown();
	void ExternalUp(int up);
	void ExternalHover();
	void ExternalDir(double p, double r, double h, double t);
	void Calibration();
	void Move(double x, double y, double z);
	void RotateHeading(double deg);
	void ChangeHeading(double deg);
	void SetId(int id) { _id = id; }
	int GetId() { return _id; }
	void Land(int seconds = 10) { landCounter = seconds; _landTime.start(); }
	bool IsFinishTheMission() { return eom; }
	void setIsMainRobot(bool value);
	void setMainRobot(Robot* robot);
	Robot *mainRobot;
	void SetMission(Mission* m);
	void StepMission(float* currentPos);
	void saveTime();
	void SetSyncVariable(bool* syncV, bool* ready)
	{
		_syncVariable = syncV;
		_readyToContinue = ready;
	}

	void Read();

	bool IsWaiting();
	void CanContinue();
	float* currentPos;
protected:
	bool nextLeg();
private:
	QuadSerial* qs;
	Controller control;
	float* target;
	float *home;
	float* original;
	int callbackCount;
	int packetCount;
	float last_time;
	bool haveHome;
	float 	height;
	float* currentAngle;
	stoper::CTimer _time;
	bool firsttimerTime;
	int _id;
	stoper::CTimer _landTime;
	int landCounter;
	double desiredHeading;
	stoper::CTimer _intervalTime;
	stoper::CTimer _processTime;
	stoper::CTimer manualTime;
	bool isMainRobot = false;
	bool isShutdown;
	ThreadLock updateLock;

	Mission* _mission;
	LegKind commandE;
	int legId;
	Leg* currentLeg;
	float* lastFwd;
	bool eom;
	bool firstUpdateMission;
	bool* _syncVariable;
	bool* _readyToContinue;
};


