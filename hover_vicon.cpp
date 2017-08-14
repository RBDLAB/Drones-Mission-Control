#include <stdio.h>
#include <stdlib.h>
#include <chrono>
#include <conio.h>

#include "NatNetTypes.h"
#include "NatNetClient.h"
#include "QuadSerial.h"

#include "Controller.h"
#include "NetworkClient.h"``
#include "Robot.h"
#include "timer.h"
#include <math.h>
#include <cmath>
#include "Swarm.h"
using namespace std;

#define NROBOTS 2
#define STATION_NUMBERS 1
int indxSwap = 0;
bool optitrack = true;
bool _calibration = true;
bool DegelCont = false;
bool completeStations = false;
bool preStations = true;
Robot robot[NROBOTS];




float *PostionNow = new float[3];
float *PostionNowSwap = new float[3];


float *target = new float[3];
float *home = new float[3];

stoper::CTimer markerTime;
stoper::CTimer allMarkerTime;
bool firstMarkerStep = true;
bool firstAllMarkerStep = true;

int startIndex = 0;
Swarm* symaSwarm = NULL;
int callbackCount = 0;
int packetCount = 0;
float last_time;
const int X = 0;
const int Y = 1;
const int Z = 2;
const int H = 3;
const int Nrobots = NROBOTS;

stoper::CTimer mtime;
stoper::CTimer mtime2;
stoper::CTimer solimanT;
int someCounter = 0;
bool ready = false;
Vector3 stations[STATION_NUMBERS];
bool haveStations[STATION_NUMBERS];


void SimpleTask2(Mission* m1, double alt, double d, int n) {
	int legnum = 0;
	Angle zero(0, 0, 0);
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
}

void StabilizeTask(Mission* m1, double alt, int n) {
	int legnum = 0;
	Angle zero(0, 0, 0);

	m1->AddLeg(new Leg(LegKind::TakeOff, 0.1, legnum++));
	for (double i = 2; i < alt * 10; i++) {
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, i/10), zero, legnum++));
	}

	for (int i = 0; i < n; i++) {
		m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	}
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 1), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.9), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.8), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.7), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.6), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.5), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.4), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.3), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
	m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.2), zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
}
void SimpleTask(Mission* m1, double alt, int n) {
	int legnum = 0;
	Angle zero(0, 0, 0);
		m1->AddLeg(new Leg(LegKind::TakeOff, 0.1, legnum++));
		for (double i = 2; i < alt * 10; i++) {
			m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, i / 10), zero, legnum++));
		}

		for (int i = 0; i < 200; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.05, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.1, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.15, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.2, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.25, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.3, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.35, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.4, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.45, 0, alt), zero, legnum++));
		for (int i = 0; i < 200; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.45, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.4, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.35, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.3, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.25, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.2, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.15, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.1, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(-0.05, 0, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, alt), zero, legnum++));


		m1->AddLeg(new Leg(LegKind::Position, Vector3(0,-0.05, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0,-0.1, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0,-0.15, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.2,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.25, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.3,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.35,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.4,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.45,  alt), zero, legnum++));
		for (int i = 0; i < 200; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.45, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.4,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.35, alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.3,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.25,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.2,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.15,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.1,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, -0.05,  alt), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, alt), zero, legnum++));
		for (int i = 0; i < 600; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 1.3), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 1.2), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 1.1), zero, legnum++));
		for (int i = 0; i < 50; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 1), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.9), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.8), zero, legnum++));
		for (int i = 0; i < 50; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.7), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.6), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.5), zero, legnum++));
		for (int i = 0; i < 50; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.4), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.3), zero, legnum++));
		m1->AddLeg(new Leg(LegKind::Position, Vector3(0, 0, 0.2), zero, legnum++));
		for (int i = 0; i < 50; i++) {
			m1->AddLeg(new Leg(LegKind::Stabilize, 30, legnum++));
		}
	m1->AddLeg(new Leg(LegKind::OFF, legnum++));

}




double altitude = 1.0;
Vector3 station1(0.760, -1.786, -0.350 + altitude);
Vector3 station2(0.574, 0.601, -0.391 + altitude);
Vector3 station3(-1.342, 0.538, -0.343 + altitude);
Vector3 station4(-1.161, -1.910, -0.305 + altitude);
Vector3 station5(-0.261, -0.714, -0.352 + altitude);

void goHome(Mission* m1, Vector3 home, int& legnum)
{
	Angle zero(0, 0, 0);
	m1->AddLeg(new Leg(LegKind::Position, home,zero, legnum++));
	m1->AddLeg(new Leg(LegKind::Stabilize, 1000.0, legnum++));
}
Vector3 calculateCenter()
{
	Vector3 sum(0, 0, 0);
	for (size_t i = startIndex; i < NROBOTS; i++)
	{
		sum.x += stations[i].x;
		sum.y += stations[i].y;
		sum.z += stations[i].z;
	}
	int n = NROBOTS - startIndex;
	sum.x /= n;
	sum.y/= n;
	sum.z /= n;
	return sum;
}



int Squ(short a)
{
	return a*a;
}




long globalFC = 0;
long lastFramecounter;
void funcPreStations()
{
	for (size_t i = 0; i < STATION_NUMBERS; i++)
	{
		haveStations[i] = false;
	}
}
bool funcCompleStations(int index,float* pos)
{
	double Epsilon = 0.0000000000001;
	if (abs(pos[0]) <= Epsilon && abs(pos[1]) <= Epsilon && abs(pos[2]) <= Epsilon)
		return false;
	if (startIndex <= index && index <= STATION_NUMBERS)
	{
		printf("Index %d\n", index);
		stations[index] = Vector3(pos[1], pos[0], pos[2]);
		haveStations[index] = true;
		bool result = true;
		for (size_t i = startIndex; i < STATION_NUMBERS; i++)
		{
			result &= haveStations[i];
		}
		if (result)
		{
			for (size_t i = startIndex; i < STATION_NUMBERS; i++)
			{
				printf("Base[%d] = (%lf,%lf,%lf)\n", i,stations[i].x, stations[i].y, stations[i].z);
			}
			
		}
		return result;
	}
	else
		return false;
}
void rigidBodyCallback(int index, int id, float *pos, float *qrot, long frameCounter)
{
	try{
		PostionNow = pos;
		if (preStations)
		{
			preStations = false;
			funcPreStations();
		}
		if (!completeStations)
		{
			completeStations = funcCompleStations(id-1, pos);
		}
		if (!ready) return;
	
	if (lastFramecounter != frameCounter)
	{
		lastFramecounter = frameCounter;
	}
	
	if (!(pos[0] == 0 && pos[1] == 0 && pos[2] == 0))
	{
		if (startIndex <=id && id <= NROBOTS) {

			mtime.start();
			symaSwarm->UpdateMission(id, pos, qrot, frameCounter);

			

		}
		else
		{
		}
	}
	}
	catch (...)
	{
		printf("Exception Call");
	}

}
void markers_callback(MarkersList mList, long frameCounter)
{ 
		if (firstMarkerStep)
		{
			firstMarkerStep = false;
			markerTime.start();
		}
}
void all_markers_callback(MarkersList mList, long frameCounter)
{
		if (firstAllMarkerStep)
		{
			firstAllMarkerStep = false;
			allMarkerTime.start();
		}
}
int _up = 1000.0;
NetworkClient* client;

int main(int argc, const char* argv[]){

	mtime.start();
	mtime2.start();
	solimanT.start();
	float state[6] = { 0.0, 1.0, 2.0, 3.0, 4.0, 5.0 }; 
	float xyzTarget[3] = { 2, 2, 2 };
	symaSwarm = new Swarm();
	for (int i = startIndex; i < NROBOTS; i++)
	{ 
		if (0 == i )
			robot[i].OpenSerial(4);   
		if (1 == i)
			robot[i].OpenSerial(5);
		if (2 == i)
			robot[i].OpenSerial(7);
		if (3 == i)
			robot[i].OpenSerial(6);
		if (4 == i)
			robot[i].OpenSerial(7);
		if (5 == i)
			robot[i].OpenSerial(16);
		if (6 == i)
			robot[i].OpenSerial(16);
		if (7 == i)
			robot[i].OpenSerial(18);
		if (8 == i)
			robot[i].OpenSerial(18);
		if (9 == i)
			robot[i].OpenSerial(11);
	}
	Sleep(2000);	 //<<--  very important don't touch
	for (int i = startIndex; i < NROBOTS; i++)
		robot[i].ExternalDir(1500, 1500, 1000, 1500);
	if (optitrack)
	{
		client = new NetworkClient(&rigidBodyCallback, &markers_callback, &all_markers_callback);
		client->Initialize("127.0.0.1", "127.0.0.1", 1511, 1510, true);



	}

	Mission* m1[NROBOTS];
	for (int i = startIndex; i < NROBOTS; i++)
	{

		robot[i].SetPid(X, -0.25, -0.0, -0.15);
		robot[i].SetPid(Y, -0.25, -0.0, -0.15);
		robot[i].SetPid(Z, 1.5, 0.0, 0.75);
		robot[i].SetPid(H, -0.9, -0.0, -0.125);
		robot[i].SetId(i + 1);
		m1[i] = new Mission("Test Mission");

	}
	while (!completeStations)
	{
		Sleep(10);
	}
	 // define mission for  each robot
	for (int i = startIndex; i < NROBOTS; i++)
	{
		if (i == 0) {
			SimpleTask(m1[i], 1.3, 800);
			robot[0].setIsMainRobot(true);
		}
		else {
			if (i == 1) {
				SimpleTask2(m1[i], 0.3, 0.5, 100);
				robot[1].setMainRobot(&robot[0]);
			}
			else {
				if (i == 2) {
					SimpleTask2(m1[i], 0.3, 0.5, 100);
					robot[2].setMainRobot(&robot[0]);
				}
				else {
					if (i == 3) {
						SimpleTask2(m1[i], 0.3, 0.5, 100);
					}else{
						if (i == 4) {
							SimpleTask2(m1[i], 0.3, 0.5, 100);
						}
					}
				}
			}
		}
		
		robot[i].SetMission(m1[i]);
	}
		symaSwarm->AddRobot(robot, NROBOTS);

	printf("press  'Esc' to exit \n");
	int throttle = 0;
	int p = 128;
	int r = 128;
	int y = 128;
	bool picture = false;
	bool video = false;


	for (int j = startIndex; j < NROBOTS; j++)
		{
			robot[j].ExternalDir(1500, 1500, 1000, 1500);

			Sleep(1);
		}
	Sleep(1000);
	if (_calibration)
	{
		for (int i = 0; i < Nrobots; i++)
			robot[i].Calibration();
	}
	printf("Calibration\n");
	printf("Ready, press to start\n");
	char key_code = getch();
	for (int j = startIndex; j < NROBOTS; j++) {
		robot[j].ExternalDir(1500, 1500, 1000, 1500);
	}

	last_time = clock();

	Sleep(200);
	float *original = new float[3];
	
	float speed = 1.0;
	float radius = 0.6;
	float Zheight = 0.3;
	int goHome = 0;
	bool circle = false;
	bool stand = true;
	bool haveOrigin[3] =  { false, false,false };

	stoper::CTimer _time;
	_time.start();
	int direction[4];
	direction[0] = direction[1] = direction[2] = 1500;
	direction[3] = 1250;

	bool test = false;
 
	bool calibrate = false;
	bool stop = false;
	
	Sleep(100);
	ready = true;
	symaSwarm->SetReady(true);
	while (!stop)
	{
		try {
			Sleep(10);
			for (int i = startIndex; i < Nrobots; i++)
			{
				if (robot[i].IsEnable())
				{
					if (!haveOrigin[i])
					{
						robot[i].GetOrigen(original[0], original[1], original[2]);
						haveOrigin[i] = true;
					}
					if (!goHome)
					{
					}
				}
			}
			if (kbhit())
			{
				printf("kbhit\n");
				switch (char c = getch())
				{
				case 'q':
					robot[0].SetTarget(-1.016503,0.711986,0.419277);
					float* currentPos;
					currentPos[0] = -1.016503;
					currentPos[1] = 0.711986;
					currentPos[2] = 0.419277;
					robot[0].StepMission(currentPos);
					
					break;
				case 'e':
					original[2] -= 0.05;
					break;
				case 'w':
					original[0] += 0.05;
					break;
				case 's':
					original[0] -= 0.05;
					break;
				case 'a':
					original[1] -= 0.05;
					break;
				case 'd':
					original[1] += 0.05;
					break;
				case 'r':
					radius += 0.1;
					break;
				case 't':
					radius -= 0.1;
					break;
				case 'f':
					speed += 0.01;
					break;
				case 'g':
					speed -= 0.01;
					break;
				case 'h':
					target[0] = home[0];
					target[1] = home[1];
					goHome = 1;
					cout << "On My Way Home!" << endl;
					break;
				case 'l':
				case 'L':
					for (int i = 0; i < Nrobots; i++)
						robot[i].Land();
					break;
				case';':
				case':':
					for (int i = 0; i < Nrobots; i++)
						robot[i].RotateHeading(45.0);
					break;
				case'k':
				case'K':
					for (int i = 0; i < Nrobots; i++)
						robot[i].RotateHeading(-45.0);
					break;
				case'p':
				case'P':
					for (int i = 0; i < Nrobots; i++)
						robot[i].Move(1, 0, 0);
					break;
				case'.':
				case'v':
					for (int i = 0; i < Nrobots; i++)
						robot[i].Move(-1, 0, 0);
					break;
				case 'u':
					if (_up > 1000)
						_up--;
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].ExternalUp(_up);
					break;
				case 'U':
					if (_up < 2000)
						_up++;
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].ExternalUp(_up);
					break;
				case 'o':
				case 'O':
					_up = 1350;
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].ExternalHover();
					break;
				case 'c':
				case 'C':
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].Calibration();
					break;
				case '6':
					if (direction[2] < 2000)
						direction[2]++;
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].ExternalDir(direction[0], direction[1], direction[3], direction[2]);
					break;
				case '4':
					if (direction[2] > 1000)
						direction[2]--;
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].ExternalDir(direction[0], direction[1], direction[3], direction[2]);
					break;
				case '2':
					if (direction[0] < 2000)
						direction[0]++;
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].ExternalDir(direction[0], direction[1], direction[3], direction[2]);
					break;
				case '8':
					if (direction[0] > 1000)
						direction[0]--;
					if (!optitrack)
						for (int i = 0; i < Nrobots; i++)
							robot[i].ExternalDir(direction[0], direction[1], direction[3], direction[2]);
					break;
				case 27:
				
					ready = false;
					symaSwarm->SetReady(false);
					
					for (int i = 0; i < Nrobots; i++)
					{
						
						robot[i].ShutDown();
						
					}
					stop = true;
					ready = false;
					symaSwarm->SetReady(false);
					Sleep(10);
					delete client;
					Sleep(10);
					return 1;
					break;
				default:
					printf("different\n");
				}
				try {
					if (!optitrack)
						robot[0].saveTime();
				}
				catch (...)
				{
					printf("Exception robot[0].saveTime()");
				}
			}
			else
			{
			}


		}
		catch (...)
		{
			printf("while");
		}
	}

}
