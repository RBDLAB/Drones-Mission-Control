#pragma once
#include <map>
#include "Robot.h"
class Swarm
{
public:
	Swarm();
	~Swarm();
	void AddRobot(Robot* r, int n);
	void UpdateMission(int id, float* pos, float* qrot, long globalFC);
	void SetReady(bool val) { ready = val; }
private:
	bool AllWaiting();
	bool AllReleased();
	bool AllReady();
	bool OneOrMoreWaiting();
	bool canRelease;
	bool doSync;

	std::map<int, Robot*> robots;
	std::map<int, bool*> syncMap;
	std::map<int, bool*> readyMap;
	bool ready;
	int nRobots;
	long frameCounter;
};

