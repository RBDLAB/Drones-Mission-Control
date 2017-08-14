#include "Mission.h"


Mission::Mission(std::string name)
{
	missionName = name;
}


Mission::~Mission()
{
	legsList.clear();
}
void Mission::AddLeg(Leg* leg)
{
	legsList[leg->_id] = leg;
}
void Mission::SetLandHeight(double val)
{
	std::map<int, Leg*>::iterator itr;
	for (itr = legsList.begin(); itr != legsList.end(); itr++)
	{
		itr->second->SetLandHeight(val);
	}
}
