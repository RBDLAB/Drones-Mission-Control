#include <stdio.h>
#include <cstring>
#include <thread>
#include "NatNetTypes.h"
#include "NatNetClient.h"
#include <map>
#include "Utilities.h"
#ifndef NatNetClient_h
#define NatNetClient_h
#define DEBUG_PRINT 0


using namespace std;

//struct Name
//{
//	int rb;
//	int markerIndex;
//};
//struct Marker
//{
//	int id;
//	double x;
//	double y;
//	double z;
//	Name oldName;
//	Name newName;
//
//};
typedef map<int, Marker> MarkersList;
class NetworkClient
{
public:
	NatNetClient *client;
	int CreateClient(char hostIP[128], char clientIP[128], int dataPort, int commandPort);

	bool life;
	bool waitMe;
	//std::thread* t;
	std::thread::id	   t_id;
public:
	NetworkClient(void(*rigidBodyCallback)(int index, int id, float *position, float *qrot, long frameCounter), void(*markersCallback)(MarkersList list, long frameCounter), void(*mallMrkersCallback)(MarkersList list, long frameCounter));
	~NetworkClient();

	//int Initialize(char hostIP[128], char clientIP[128], int dataPort = 1511, int commandPort = 1510, , bool unicast=false);
	void Initialize(char hostIP[128], char clientIP[128], int dataPort = 1511, int commandPort = 1510, bool unicast = false);
	void start();
	void(*rigidBodyCallback)(int index, int id, float *position, float *qrot, long frameCounter);
	void(*markersCallback)(MarkersList list, long frameCounter);
	void(*allMarkersCallback)(MarkersList list, long frameCounter);
	static void work(NetworkClient *netClient, char hostIP[128], char clientIP[128], int dataPort, int commandPort, bool unicast);
	//
	MarkersList markerList;
	MarkersList allmarkerList;
};
#endif					