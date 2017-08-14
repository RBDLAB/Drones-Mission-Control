#pragma once
#include <math.h>
#include <map>
struct Name
{
	int rb;
	int markerIndex;
};
struct Marker
{
	int id;
	double x;
	double y;
	double z;
	Name oldName;
	Name newName;

};
struct MarkerPos
{
	int id;
	double time;
	long frame;
	double x;
	double y;
	double z;
};
struct DataContainer
{
	int n;
	MarkerPos* markers;
};


typedef std::map<long, DataContainer> markersData;
class Utils
{
public:
	static float *eulerAnglesZYX(float *q)
	{
		//normalize q
		float *n = new float[4];
		float q_len = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

		for (int i = 0; i < 4; i++)
			n[i] = q[i] / q_len;

		float *euler = new float[3];
		euler[0] = atan2(2 * (n[3] * n[0] - n[1] * n[2]), n[0] * n[0] + n[1] * n[1] - n[2] * n[2] - n[3] * n[3]);
		euler[1] = asin(2 * (n[1] * n[3] + n[2] * n[0]));
		euler[2] = atan2(2 * (n[1] * n[0] - n[2] * n[3]), n[0] * n[0] - n[1] * n[1] - n[2] * n[2] + n[3] * n[3]);

		return euler;
	}
	static void readMarkersPosFile(std::string fileName, markersData& markers_container)
	{
		FILE* ff = fopen(fileName.c_str(), "r");
		markers_container.clear();
		if (ff == NULL) return;
		while (!feof(ff))
		{
			float dt;
			float frame;
			float nMarkers;
			fscanf(ff, "dt: %f frame: %f nMarkers: %f", &dt, &frame, &nMarkers);
			DataContainer container;
			MarkerPos* line = new MarkerPos[(int)nMarkers];
			container.markers = line;
			container.n = nMarkers;
			for (int i = 0; i < nMarkers; i++)
			{

				float x, y, z;
				MarkerPos marker;
				marker.id = i;
				marker.frame = frame;
				marker.time = dt;
				fscanf(ff, " (%f,%f,%f) ", &marker.x, &marker.y, &marker.z);// &(line[i].x), &(line[i].y), &(line[i].z));
				printf("(%f,%f,%f) ", marker.x, marker.y, marker.z);// (line[i].x), (line[i].y), (line[i].z));
				line[i] = marker;
			}
			markers_container[(long)frame] = container;
		}

	}
};