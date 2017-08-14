#include "Leg.h"

Leg::Leg()
{
	canFinalize = true;
	_steps = 1;
}


Leg::~Leg()
{
}
Leg::Leg(LegKind kind, Vector3 pos, Angle dir, int id)
{
	canFinalize = true;
	_kind = kind;
	_desiredPosition = pos;
	_desiredAngle = dir;
	_id = id;
	_steps = 1;
	SetEpsilonDefault();
}
Leg::Leg(LegKind kind, Vector3 pos, int steps, bool linear, int id)
{
	//this constructor is for Path
	canFinalize = true;
	_kind = kind;
	_desiredPosition = pos;
	_steps = steps;
	_id = id;
	SetEpsilonDefault();
}
Leg::Leg(LegKind kind, double val, int id)
{
	canFinalize = true;
	_steps = 1;
	_kind = kind;
	switch (_kind)
	{
	case LegKind::ChangeHeight:
		_desiredPosition = Vector3(0, 0, val);
		break;
	case LegKind::Direction:
	case LegKind::DirectionRel:
		_desiredPosition = Vector3(0, 0, 0);
		_desiredAngle = Angle(val, 0, 0);
		break;
	case LegKind::Forward:
		_desiredPosition = Vector3(val, 0, 0);
		_desiredAngle = Angle(0, 0, 0);
		break;
	case LegKind::Land:
		break;
	case LegKind::OFF:
		_desiredTime = 1000.0; 
		break;
	case LegKind::Right:
		_desiredPosition = Vector3(0, val, 0);
		_desiredAngle = Angle(0, 0, 0);
		break;
	case LegKind::Stabilize:
	case LegKind::Wait:
		_desiredTime = val;
		break;
	case LegKind::StabilizeSync:
	case LegKind::WaitSync:
		canFinalize = false;
		_desiredTime = val;
		break;
	case LegKind::TakeOff:
		_desiredPosition = Vector3(0, 0, val);
		_desiredAngle = Angle(0, 0, 0);
		break;
	default:
		throw "Incorrect Constructor for this LegKind!!";
		break;
	}
	_id = id;
	SetEpsilonDefault();
}
Leg::Leg(LegKind kind, int id)
{
	if ((kind != LegKind::Land) && (kind != LegKind::OFF))
		throw "Incorrect Constructor for this LegKind!!";
	_id = id;
	_kind = kind;
	_steps = 1;
	SetEpsilonDefault();

}
void Leg::SetEpsilonDefault()
{
	EpsilonPos = 0.56;
	EpsilonForward = 0.10;
	EpsilonRight = 0.025;
	EpsilonZ = 0.15;
	EpsilonDeg = 1.0;
	EpsilonTime = 16.0;

	haveInitPos = false;
	haveInitAng = false;

}


std::string Leg::GetKind()
{
	std::string kind;
	switch (_kind)
	{
	case LegKind::ChangeHeight:
		kind = "ChangeHeight";
		break;
	case LegKind::Direction:
		kind = "Direction";
		break;
	case LegKind::DirectionRel:
		kind = "DirectionRel";
		break;
	case LegKind::Forward:
		kind = "Forward";
		break;
	case LegKind::Land:
		kind = "Land";
		break;
	case LegKind::OFF:
		kind = "Land";
		break;
	case LegKind::Path:
		kind = "Path";
		break;
	case LegKind::Position:
		kind = "Position";
		break;
	case LegKind::Right:
		kind = "Right";
		break;
	case LegKind::Stabilize:
		kind = "Stabilize";
		break;
	case LegKind::StabilizeSync:
		kind = "StabilizeSync";
		break;
	case LegKind::Stop:
		kind = "Stop";
		break;
	case LegKind::TakeOff:
		kind = "TakeOff";
		break;
	case LegKind::Wait:
		kind = "Wait";
		break;
	case LegKind::WaitSync:
		kind = "WaitSync";
		break;
	default:
		break;
	}
	return kind;
}
void Leg::SetCurrentPos(double x, double y, double z)
{
	if (!haveInitPos)
	{
		haveInitPos = true;
		_initPos = Vector3(x, y, z);
		_time.start();
	}
	_currentPos = Vector3(x, y, z);
}
void Leg::SetCurrentAngDeg(double yaw, double pitch, double roll)
{
	if (!haveInitAng)
	{
		haveInitAng = true;
		_initAngle = Angle(yaw, pitch, roll);
	}
	if (yaw < -90)
		yaw += 360.0;
	_currentAngle = Angle(yaw, pitch, roll);
}
void Leg::SetCurrentAngRad(double yaw, double pitch, double roll)
{
	SetCurrentAngDeg(Util::degrees(yaw), Util::degrees(pitch), Util::degrees(roll));
}
bool Leg::EoLeg()
{
	bool result = false;
	switch (_kind)
	{
	case LegKind::ChangeHeight:
	case LegKind::TakeOff:
	case LegKind::Land:
	
		if (GetError() < EpsilonZ)
		{
			readyTonext = true;
			result = true;
		}
		break;
	case LegKind::Direction:
	case LegKind::DirectionRel:
		if (GetError() < EpsilonDeg)
		{
			readyTonext = true;
			result = true;
		}
		break;
	case LegKind::Forward:
		if (GetError() < EpsilonForward)
		{
			readyTonext = true;
			result = true;
		}
		break;
	case LegKind::OFF:		
	case LegKind::Stabilize:
	case LegKind::Wait:
		if (GetError() < EpsilonTime)
		{
			readyTonext = true;
			result = true;
		}
		break;
	case LegKind::Path:
		break;
	case LegKind::Position:
		if (GetError() < EpsilonPos)
		{
			result = true;
			readyTonext = true;
		}
		break;
	case LegKind::Right:
		if (GetError() < EpsilonRight)
		{
			readyTonext = true;
			result = true;
		}
		break;
	case LegKind::StabilizeSync:
		if (GetError() < EpsilonDeg)
		{
			readyTonext = true;
			result = true && canFinalize;
		}
		break;
	case LegKind::Stop:
		break;
	case LegKind::WaitSync:
		if (GetError() < EpsilonTime)
		{
			readyTonext = true;
			result = true && canFinalize;
		}
		break;
	default:
		break;
	}

	return result;
}
double Leg::GetDirectionRelError()
{
	if (_initAngle.yaw + _desiredAngle.yaw > 0 && _currentAngle.yaw < 0)
		_currentAngle.yaw += 360.0;
	else
		if (_initAngle.yaw + _desiredAngle.yaw < 0 && _currentAngle.yaw > 0)
			_currentAngle.yaw -= 360.0;
	double dYaw = _currentAngle.yaw - _initAngle.yaw;
	return  abs(_desiredAngle.yaw - dYaw);
}
double Leg::GetDirectionError()
{
	double dYaw = _currentAngle.yaw - _desiredAngle.yaw;
	if (dYaw > 180)
		dYaw -= 360;
	else
		if (dYaw < -180)
			dYaw += 360;

	return abs(dYaw);
}
double Leg::GetPositionError()
{
	double dx = _currentPos.x - _desiredPosition.x;
	double dy = _currentPos.y - _desiredPosition.y;
	double dz = _currentPos.z - _desiredPosition.z;
	return sqrt(dx*dx + dy*dy + dz*dz);
}
double Leg::GetTakeOffError()
{
	double dz = _currentPos.z - _initPos.z;
	return abs(_desiredPosition.z - dz);
}
double Leg::GetChangeAltError()
{
	if (_initPos.z + _desiredPosition.z < landHeight)
		_desiredPosition.z = _initPos.z - landHeight;

	double dz = _currentPos.z - _initPos.z;
	return abs(_desiredPosition.z - dz);
}
double Leg::GetLandError()
{
	double dz = _currentPos.z - landHeight;
	return abs(dz);
}
double Leg::GetRelPosError()
{
	double dx = _currentPos.x - _initPos.x;
	double dy = _currentPos.y - _initPos.y;
	double Rdx = abs(_desiredPosition.x);
	double Rdy = abs(_desiredPosition.y);
	return (sqrt(Rdx*Rdx + Rdy*Rdy) - sqrt(dx*dx + dy*dy));
}
double Leg::GetTimeError()
{
	double error = 0.0;
	error = (_desiredTime - _time.getTime());
	if (error < 0)
		error = 0;
	return 	  error;
}

double Leg::GetError()
{
	double error = -1000.0;
	switch (_kind)
	{
	case LegKind::ChangeHeight:
		error = GetChangeAltError();
		break;
	case LegKind::Direction:
		error = GetDirectionError();
		break;
	case LegKind::DirectionRel:
		error = GetDirectionRelError();
		break;
	case LegKind::Forward:
	case LegKind::Right:
		error = GetRelPosError();
		break;
	case LegKind::Land:
		error = GetLandError();
		break;
	case LegKind::Path:
		break;
	case LegKind::Position:
		error = GetPositionError();
		break;
	case LegKind::OFF:
	case LegKind::Stabilize:
	case LegKind::Wait:
	case LegKind::StabilizeSync:
	case LegKind::WaitSync:
		error = GetTimeError();
		break;
	case LegKind::Stop:
		break;
	case LegKind::TakeOff:
		error = GetTakeOffError();
		break;
	default:
		break;
	}
	return error;
}