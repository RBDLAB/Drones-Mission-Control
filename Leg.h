#pragma once
#include "Util.h"
#include <string>
#include "timer.h"
enum LegKind
{
	DirectionRel,
	Direction,
	Position,
	Stabilize,
	TakeOff,
	Land,
	ChangeHeight,
	Forward,
	Right,
	Wait,
	WaitSync,
	OFF,
	StabilizeSync,
	Path,
	Stop,
	MFSync
};
class Leg
{
public:
	Leg();
	Leg(LegKind kind, Vector3 pos, Angle dir, int id);
	Leg(LegKind kind, double val, int id);
	Leg(LegKind kind, int id);
	Leg(LegKind kind, Vector3 pos, int steps,bool linear, int id);

	void SetCurrentPos(double x, double y, double z);
	void SetCurrentAngRad(double yaw, double pitch, double roll);
	void SetCurrentAngDeg(double yaw, double pitch, double roll);

	double GetError();
	bool EoLeg();
	void SetEpsilonDefault();
	void Print();
	~Leg();
	void SetID(int id) { _id = id; }
	std::string GetKind();
	LegKind GetKindE() { return _kind; }
	void SetLandHeight(double val) { landHeight = val; }
	LegKind _kind;
	void SetCanFinalize(bool val) { canFinalize = val; }
	bool IsReadyToNext() { return readyTonext; }
	void Reset() { readyTonext = false; }

	Angle _desiredAngle;
	Vector3 _desiredPosition;
	int _id;
protected:
	double GetDirectionRelError();
	double GetPositionError();
	double GetTakeOffError();
	double GetChangeAltError();
	double GetLandError();
	double GetRelPosError();
	double GetTimeError();
	double GetDirectionError();





	Angle _currentAngle;
	Vector3 _currentPos;
	double EpsilonPos;
	double EpsilonForward;
	double EpsilonRight;
	double EpsilonZ;
	double EpsilonDeg;
	double EpsilonTime;
	Angle _initAngle;
	Vector3 _initPos;
	bool haveInitPos = false;
	bool haveInitAng;
	double landHeight;
	stoper::CTimer _time;
	double _desiredTime;
	bool canFinalize;
	bool readyTonext;
	int _steps;
	bool _linear;
 
};

