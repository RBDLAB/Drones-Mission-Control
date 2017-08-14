#include "Swarm.h"


Swarm::Swarm()
{
	canRelease = false;
	doSync = false;
	ready = false;
	nRobots = 0;
}


Swarm::~Swarm()
{
	for (auto& r : syncMap)
	{
		delete (r.second);
	}
	for (auto& r : robots)
	{
		delete (r.second);
	}
}
void Swarm::AddRobot(Robot* r, int n)
{
	nRobots = n;
	for (int i = 0; i < n; i++)
	{
		int id = r[i].GetId();
		robots[i] = &(r[i]);
		printf("id=%d,%d\n", id, robots[i]->GetId());
		syncMap[id] = new bool;
		*(syncMap[id]) = false;
		readyMap[id] = new bool;
		*(syncMap[id]) = false;
		robots[i]->SetSyncVariable(syncMap[id], readyMap[id]);
	}
}
void Swarm::UpdateMission(int id, float* pos, float* qrot, long globalFC)
{
	frameCounter = globalFC;
	if (!ready) return;
	if (!(pos[0] == 0 && pos[1] == 0 && pos[2] == 0))
	{
		try
		{
			if (id <= nRobots) {
				robots[id - 1]->UpdateMission(pos, qrot, globalFC);
			}
			else
			{
				printf("rb id:%d\n", id);
			}
		}
		catch (...)
		{
			printf("Exception Call");
		}
	}

	if (OneOrMoreWaiting() && !doSync)
	{
		doSync = true;
		canRelease = false;

	}
	if (AllReady() && doSync)
	{
		canRelease = true;
	}
	if (AllWaiting() && doSync && canRelease)
	{
		doSync = false;
		canRelease = false;
		for (auto& r : syncMap)
		{
			*(r.second) = false;
		}
		for (auto& r : readyMap)
		{
			*(r.second) = false;
		}
		for (auto& r : robots)
		{
			(r.second)->CanContinue();
		}
	}


}
bool Swarm::AllWaiting()
{
	bool result = true;
	for (auto& r : syncMap)
	{
		result = result &&*(r.second);
	}
	return result;
}
bool Swarm::AllReady()
{
	bool result = true;
	for (auto& r : readyMap)
	{
		result = result &&*(r.second);
	}
	return result;
}
bool Swarm::AllReleased()
{
	bool result = true;
	for (auto& r : syncMap)
	{
		result = result && !*(r.second);
	}
	return result;
}
bool Swarm::OneOrMoreWaiting()
{
	bool result = false;
	for (auto& r : syncMap)
	{
		result = result || *(r.second);
	}
	return result;
}
