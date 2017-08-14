#pragma once
#include <math.h>

struct Geo
{
	double latitude;
	double longitude;
	double altitude;
};
struct Angle
{
	double pitch;
	double roll;
	double yaw;
	Angle() {}
	Angle(double y, double p, double r)
	{
		yaw = y;
		pitch = p;
		roll = r;

	}
};
struct Control
{
	double up;
	double forward;
	double left;
	double yaw;

};
struct EarthFrameRate // degree/s
{
	EarthFrameRate() {};
	EarthFrameRate(double p, double r, double y)
	{
		pitch_rate = p;
		roll_rate = r;
		yaw_rate = y;
	}
	double pitch_rate;
	double roll_rate;
	double yaw_rate;
};
struct BodyFrameRate	// degree/s
{
	double pDeg;
	double qDeg;
	double rDeg;
};
struct Vector3
{
	Vector3() {};
	Vector3(double _x, double _y, double _z)
	{
		x = _x;
		y = _y;
		z = _z;
	}
	virtual Vector3 operator+(Vector3 other)
	{
		Vector3 v;

		v.x = x + other.x;
		v.y = y + other.y;
		v.z = z + other.z;
		return v;
	}
	virtual Vector3 operator*(double factor)
	{
		Vector3 v;

		v.x = x * factor;
		v.y = y * factor;
		v.z = z * factor;
		return v;
	}
	double x;
	double y;
	double z;
};
class Util
{
public:
	Util();
	~Util();
	static double radians(double deg)
	{
		return acos(0.0)*deg / 90.0;
	}
	static double degrees(double rad)
	{
		return 90.0*rad / acos(0.0);
	}

};

