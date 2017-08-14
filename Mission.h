#pragma once
#include "Leg.h"
#include <map>
#include <string>
class Mission
{
public:
	Mission(std::string name);
	~Mission();
	void SetLandHeight(double val);
	void AddLeg(Leg* leg);
	std::string missionName;
	std::map<int, Leg*> legsList;
};

