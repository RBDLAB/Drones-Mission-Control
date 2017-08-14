#include "Robot.h"
#include <math.h>
#include "ThreadLock.h"

#define _SYMA
double PI_2 = acos(0);
Robot::Robot()
{
	target = new float[3];
	home = new float[3];
	original = new float[3];
	currentPos = new float[3];
	lastFwd = new float[3];
	firstUpdateMission = true;

	callbackCount = 0;
	packetCount = 0;
	last_time;
	//float state[6] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 }; // getS();
	//float xyzTarget[3] = { 2, 2, 2 };
	haveHome = false;
	height = 0;

	control.PIDs[0] = new PID(0.08, 0.00, 0.175);	 
	control.PIDs[1] = new PID(0.08, 0.00, 0.175);
	control.PIDs[2] = new PID(1.0, 0.0, 0.4); 
	landCounter = -1;
	isShutdown = false;
	firsttimerTime = true;
	_time.start();
	desiredHeading = 0;
	_intervalTime.start();
	_processTime.start();
	updateLock.Unlock();
	_mission = NULL;
	legId = -1;
	currentLeg = NULL;
	commandE = LegKind::Stop;
	eom = false;
}
void Robot::OpenSerial(int port)
{	
	qs = new QuadSerial(port);
}


Robot::~Robot()
{
	delete qs;
	qs = NULL;
}
void Robot::saveTime()
{
	manualTime.getTimeAndRelease();
}
void Robot::UpdateMission(float* pos, float* qrot, long globalFC)
{
	if (eom)
	{
		ShutDown();
		return;
	}
	if (firstUpdateMission)
	{
		firstUpdateMission = false;

	}

	_processTime.start();
	manualTime.start();
	double interval = _intervalTime.getTimeAndRelease();

	float qBody[4];
	if (firsttimerTime)
	{
		firsttimerTime = false;
		_time.start();
	}

	qBody[0] = -qrot[3];
	qBody[1] = qrot[2];
	qBody[2] = -qrot[1];
	qBody[3] = qrot[0];

	float *angles = Utils::eulerAnglesZYX(qBody);
	float angleX = -angles[0]; // must invert due to 180 flip above
	float angleY = angles[1];
	float angleZ = -angles[2]; // must invert due to 180 flip above


	currentPos[0] = pos[1]; // obj.x = y
	currentPos[1] = pos[0]; // obj.y = x
	currentPos[2] = pos[2]; // obj.z = z
	currentPos[3] = angleY;
	currentPos[4] = angleX;
	currentPos[5] = -angleZ;



	if (!haveHome)
	{

		target[0] = currentPos[0];
		target[1] = currentPos[1];
		target[2] = currentPos[2];
		home[0] = currentPos[0];
		home[1] = currentPos[1];
		home[2] = currentPos[2];

		original[0] = currentPos[0];
		original[1] = currentPos[1];
		original[2] = currentPos[2];
		haveHome = true;
	}
	if (currentLeg != NULL)
	{

	}
	double t1 = _processTime.getTimeAndRelease();
	StepMission(currentPos);
	double t2 = _processTime.getTimeAndRelease();
	if (landCounter > 0)
	{
		int t = landCounter - (_landTime.getTime() / 1000);
		if (t > 0)
		{
			double dz = (currentPos[2] - original[2]) / t;
			target[2] = original[2] - dz;
		}

		if (abs(currentPos[2] - original[2]) < 0.09)
		{
			ShutDown();
		}
	}

	float data[4];

	double headingDif = currentPos[5] - desiredHeading;
	if (abs(headingDif) > PI_2*2.0)
	{
		if (headingDif < 0)
			headingDif += 4.0 * PI_2;
		else
			headingDif -= 4.0 * PI_2;
	}

	double t3 = _processTime.getTimeAndRelease();
	if ((currentLeg->GetKind().compare("Wait") == 0) || (currentLeg->GetKind().compare("WaitSync") == 0) || (currentLeg->GetKind().compare("OFF") == 0))
	{
		data[0] = data[1] = data[3] = 1500.0;
		data[2] = 1000.0;
	}
	else
		control.computeControl(currentPos, target, data, headingDif);
	data[1] = 3000 - data[1];	



#ifdef _SYMA
	double t4 = _processTime.getTimeAndRelease();

	qs->writeSyma(data, false, false);

	double t5 = _processTime.getTimeAndRelease();
#else
	qs.writeLadyBird(data);
#endif

	callbackCount++;
	delete angles;
}
void Robot::StepMission(float* currentPos)
{
	if (legId == -1)
	{
		nextLeg();
	}
	currentLeg->SetCurrentPos(currentPos[0], currentPos[1], currentPos[2]);	   
	currentLeg->SetCurrentAngRad(currentPos[5], currentPos[3], currentPos[4]);
	*_readyToContinue = currentLeg->IsReadyToNext();

	if (currentLeg->EoLeg())
	{
		if (!nextLeg())
		{
			eom = true;
			ShutDown();
		}
	}
}

void Robot::setIsMainRobot(bool value)
{
	isMainRobot = value;
}

void Robot::setMainRobot(Robot * robot)
{
	mainRobot = robot;
}

void Robot::SetMission(Mission* m)
{
	legId = -1;
	currentLeg = NULL;
	commandE = LegKind::Stop;
	eom = false;
	firstUpdateMission = true;
	haveHome = false;
	height = 0;
	landCounter = -1;
	firsttimerTime = true;
	_mission = m;
}
void Robot::SetTarget(double x, double y, double z)
{
	target[0] = x;
	target[1] = y;
	target[2] = z;
}
void Robot::GetOrigen(float& x, float& y, float& z)
{
	x = original[0];
	y = original[1];
	z = original[2];
}
void Robot::SetPid(int xyz, double p, double i, double d)
{
	control.PIDs[xyz] = new PID(p, i, d);
}
void Robot::ShutDown()
{
	float shutdown[4];
	shutdown[0] = 1500;
	shutdown[1] = 1500;
	shutdown[3] = 1500;

#ifdef _SYMA

	shutdown[2] = 1000;
	for (int i = 0; i < 3; i++)
	{
		qs->writeSyma(shutdown, false, false);

	}

#else

	shutdown[2] = 1000;

	qs.writeLadyBird(shutdown);
#endif
	isShutdown = true;
}
void Robot::ExternalUp(int up)
{
	float shutdown[4];
	shutdown[0] = 1500;
	shutdown[1] = 1500;
	shutdown[3] = 1500;

	printf("UP = %d\n", up);

	shutdown[2] = up;


}
void Robot::ExternalHover()
{
	float over[4];
	over[0] = 1500;
	over[1] = 1500;
	over[3] = 1500;
	over[2] = 1350;



}
void Robot::ExternalDir(double p, double r, double t, double h)
{
	float dir[4];
	dir[0] = p;
	dir[1] = r;
	dir[3] = h;
	dir[2] = t;


}
void Robot::Read()
{
	qs->read();
}
void Robot::Calibration()
{
	float calibration[4];


	calibration[0] = 2000;
	calibration[1] = 2000;
	calibration[3] = 2000;
	calibration[2] = 1000;
	qs->writeSyma(calibration, false, false);

}
void Robot::RotateHeading(double deg)
{
	desiredHeading = currentPos[5] + PI_2*deg / 90.0;
}
void Robot::ChangeHeading(double deg)
{
	desiredHeading = PI_2*deg / 90.0;
}
void Robot::Move(double x, double y, double z)
{
	target[0] += y * sin(-currentPos[5]) + x * cos(-currentPos[5]);
	target[1] += y * cos(-currentPos[5]) - x* sin(-currentPos[5]);
	target[2] += z;
	lastFwd[0] = target[0];
	lastFwd[1] = target[1];
	lastFwd[2] = target[2];
}
bool Robot::nextLeg()
{
	if (isMainRobot) {
		if (_mission->legsList.size() > legId + 1)
		{
			*_syncVariable = false;
			currentLeg = _mission->legsList[++legId];
			currentLeg->Reset();
			printf("Current-Leg : %d\n", currentLeg->_id);
			//command = currentLeg->GetKind();
			commandE = currentLeg->GetKindE();
			switch (commandE)
			{
			case LegKind::Forward:
				Move(currentLeg->_desiredPosition.x, 0, 0);
				break;
			case LegKind::Right:
				break;
			case LegKind::Direction:
				ChangeHeading(currentLeg->_desiredAngle.yaw);
				break;
			case LegKind::Position:
				SetTarget(currentLeg->_desiredPosition.x, currentLeg->_desiredPosition.y, currentLeg->_desiredPosition.z);
				lastFwd[0] = target[0];
				lastFwd[1] = target[1];
				lastFwd[2] = target[2];
				break;
			case LegKind::TakeOff:
				landCounter = -1;
				SetTarget(home[0], home[1], home[2] + currentLeg->_desiredPosition.z);
				_mission->SetLandHeight(home[2]);
				lastFwd[0] = target[0];
				lastFwd[1] = target[1];
				lastFwd[2] = target[2];
				break;
			case LegKind::ChangeHeight:
				if (currentPos[2] + currentLeg->_desiredPosition.z < home[2])
					currentLeg->_desiredPosition.z = currentPos[2] - home[2];
				SetTarget(currentPos[0], currentPos[1], currentPos[2] + currentLeg->_desiredPosition.z);

				lastFwd[0] = target[0];
				lastFwd[1] = target[1];
				lastFwd[2] = target[2];
				break;
			case LegKind::Land:
				Land();
				break;
			case LegKind::StabilizeSync:
				*_syncVariable = true;
				*_readyToContinue = false;
				SetTarget(lastFwd[0], lastFwd[1], lastFwd[2]);
				break;
			case LegKind::DirectionRel:
				RotateHeading(currentLeg->_desiredAngle.yaw);
				break;
			case LegKind::WaitSync:
				*_syncVariable = true;
				*_readyToContinue = false;
				SetTarget(lastFwd[0], lastFwd[1], lastFwd[2]);
				break;
			case LegKind::Wait:
			case LegKind::Stabilize:
			case LegKind::OFF:
					SetTarget(lastFwd[0], lastFwd[1], lastFwd[2]);
				break;
			}
		}
		else
		{
			Sleep(1);
			ShutDown();
			Sleep(1);
			return false;
		}
	}
	else {
		if (mainRobot->isShutdown) {
			Sleep(1);
			ShutDown();
			Sleep(1);
			return false;
		}
		else {
			currentLeg = _mission->legsList[0];
			currentLeg->Reset();
			switch (_id) {
			case 2:
				SetTarget(mainRobot->currentPos[0], mainRobot->currentPos[1] + 0.7, mainRobot->currentPos[2]-0.07);
				break;
			case 3:
				SetTarget(mainRobot->currentPos[0] + 0.7, mainRobot->currentPos[1], mainRobot->currentPos[2]);
				break;
			case 4:
				SetTarget(mainRobot->currentPos[0] + 0.7, mainRobot->currentPos[1] + 0.7, mainRobot->currentPos[2]);
				break;
			}
		}

	}
	return true;
}
void Robot::CanContinue()
{
	currentLeg->SetCanFinalize(true);
}

