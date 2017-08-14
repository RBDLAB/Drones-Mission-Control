#include "NetworkClient.h"
#include <iostream>
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include "timer.h"

NetworkClient *currentNetworkClient;
bool firstrun = true;
stoper::CTimer myTimer;

// DataHandler receives data from the server
void __cdecl DataHandler(sFrameOfMocapData* data, void* pUserData)
{
	static long frameCounter = 0;
	//printf("DataHandler Time: %lf\n", myTimer.getTimeAndRelease());
	NatNetClient* pClient = (NatNetClient*)pUserData;
	if (firstrun) {
		firstrun = false;
		auto Thandle = GetCurrentThread();
		SetThreadPriority(Thandle, THREAD_PRIORITY_HIGHEST);
	}
	int i = 0;

#if DEBUG_PRINT
	printf("FrameID : %d\n", data->iFrame);
	printf("Timestamp :  %3.2lf\n", data->fTimestamp);
	printf("Latency :  %3.2lf\n", data->fLatency);
#endif

	// FrameOfMocapData params
	bool bIsRecording = data->params & 0x01;
	bool bTrackedModelsChanged = data->params & 0x02;
#if DEBUG_PRINT
	if (bIsRecording)
		printf("RECORDING\n");
	if (bTrackedModelsChanged)
		printf("Models Changed.\n");
#endif

	// timecode - for systems with an eSync and SMPTE timecode generator - decode to values
	int hour, minute, second, frame, subframe;
	bool bValid = pClient->DecodeTimecode(data->Timecode, data->TimecodeSubframe, &hour, &minute, &second, &frame, &subframe);
	// decode to friendly string
	char szTimecode[128] = "";
	pClient->TimecodeStringify(data->Timecode, data->TimecodeSubframe, szTimecode, 128);

#if DEBUG_PRINT
	printf("Timecode : %s\n", szTimecode);

	// Other Markers
	printf("Other Markers [Count=%d]\n", data->nOtherMarkers);
	for (i = 0; i < data->nOtherMarkers; i++)
	{
		printf("Other Marker %d : %3.2f\t%3.2f\t%3.2f\n",
			i,
			data->OtherMarkers[i][0],
			data->OtherMarkers[i][1],
			data->OtherMarkers[i][2]);
	}
	// Rigid Bodies
	printf("Rigid Bodies [Count=%d]\n", data->nRigidBodies);
#endif
	frameCounter++;
	currentNetworkClient->markerList.clear();
	currentNetworkClient->allmarkerList.clear();
	int markerIndex = 1;

	for (i = 0; i < data->nRigidBodies; i++)
	{
		try
		{
			// params
			// 0x01 : bool, rigid body was successfully tracked in this frame
			bool bTrackingValid = data->RigidBodies[i].params & 0x01;

			if (currentNetworkClient->rigidBodyCallback != NULL)
			{
				float *pos = new float[3];
				pos[0] = data->RigidBodies[i].x;
				pos[1] = data->RigidBodies[i].y;
				pos[2] = data->RigidBodies[i].z;
				float *qrot = new float[4];
				qrot[0] = data->RigidBodies[i].qx;
				qrot[1] = data->RigidBodies[i].qy;
				qrot[2] = data->RigidBodies[i].qz;
				qrot[3] = data->RigidBodies[i].qw;


				currentNetworkClient->rigidBodyCallback(i, data->RigidBodies[i].ID, pos, qrot, frameCounter);
			}
		}
		catch (...)
		{
			printf("CallBack Function Exception");
		}
#if DEBUG_PRINT
		////printf("Rigid Body [ID=%d  Error=%3.2f  Valid=%d]\n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError , data->RigidBodies[i].bTrackingValid);
		//printf("Rigid Body [ID=%d  Error=%3.2f \n", data->RigidBodies[i].ID, data->RigidBodies[i].MeanError);
		//printf("\tx\ty\tz\tqx\tqy\tqz\tqw\n");
		//printf("\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
		//	data->RigidBodies[i].x,
		//	data->RigidBodies[i].y,
		//	data->RigidBodies[i].z,
		//	data->RigidBodies[i].qx,
		//	data->RigidBodies[i].qy,
		//	data->RigidBodies[i].qz,
		//	data->RigidBodies[i].qw);

		printf("\tRigid body markers *[Count=%d]\n", data->RigidBodies[i].nMarkers);
		for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
		{
			printf("\t\t");
			if (data->RigidBodies[i].MarkerIDs)
				printf("MarkerID:%d", data->RigidBodies[i].MarkerIDs[iMarker]);
			if (data->RigidBodies[i].MarkerSizes)
				printf("\tMarkerSize:%3.2f", data->RigidBodies[i].MarkerSizes[iMarker]);
			if (data->RigidBodies[i].Markers)
				printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n",
					data->RigidBodies[i].Markers[iMarker][0],
					data->RigidBodies[i].Markers[iMarker][1],
					data->RigidBodies[i].Markers[iMarker][2]);
		}
#endif
		sRigidBodyData rb = data->RigidBodies[i];
		for (int iMarker = 0; iMarker < rb.nMarkers; iMarker++)
		{
			Marker marker;

			marker.oldName.rb = rb.ID;
			if (data->RigidBodies[i].MarkerIDs != NULL)	  // todo invalidate this data
				marker.oldName.markerIndex = data->RigidBodies[i].MarkerIDs[iMarker];
			else
				marker.oldName.markerIndex = iMarker;

			marker.x = data->RigidBodies[i].Markers[iMarker][0];
			marker.y = data->RigidBodies[i].Markers[iMarker][1];
			marker.z = data->RigidBodies[i].Markers[iMarker][2];
			marker.id = markerIndex++;
			currentNetworkClient->markerList[marker.id] = marker;

		}


	}

	if (currentNetworkClient->markerList.size() > 0)
		currentNetworkClient->markersCallback(currentNetworkClient->markerList, frameCounter);
#if DEBUG_PRINT
	// skeletons
	printf("Skeletons [Count=%d]\n", data->nSkeletons);
	for (i = 0; i < data->nSkeletons; i++)
	{
		sSkeletonData skData = data->Skeletons[i];
		printf("Skeleton [ID=%d  Bone count=%d]\n", skData.skeletonID, skData.nRigidBodies);
		for (int j = 0; j< skData.nRigidBodies; j++)
		{
			sRigidBodyData rbData = skData.RigidBodyData[j];
			printf("Bone %d\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\t%3.2f\n",
				rbData.ID, rbData.x, rbData.y, rbData.z, rbData.qx, rbData.qy, rbData.qz, rbData.qw);

			printf("\tRigid body markers [Count=%d]\n", rbData.nMarkers);
			for (int iMarker = 0; iMarker < data->RigidBodies[i].nMarkers; iMarker++)
			{
				printf("\t\t");
				if (rbData.MarkerIDs)
					printf("MarkerID:%d", rbData.MarkerIDs[iMarker]);
				if (rbData.MarkerSizes)
					printf("\tMarkerSize:%3.2f", rbData.MarkerSizes[iMarker]);
				if (rbData.Markers)
					printf("\tMarkerPos:%3.2f,%3.2f,%3.2f\n",
						data->RigidBodies[i].Markers[iMarker][0],
						data->RigidBodies[i].Markers[iMarker][1],
						data->RigidBodies[i].Markers[iMarker][2]);
			}
		}
	}
#endif

	// labeled markers
	bool bOccluded;     // marker was not visible (occluded) in this frame
	bool bPCSolved;     // reported position provided by point cloud solve
	bool bModelSolved;  // reported position provided by model solve

#if DEBUG_PRINT
	printf("Labeled Markers [Count=%d]\n", data->nLabeledMarkers);
#endif
	markerIndex = 0;
	for (i = 0; i < data->nLabeledMarkers; i++)
	{
		Marker marker;

		bOccluded = data->LabeledMarkers[i].params & 0x01;
		bPCSolved = data->LabeledMarkers[i].params & 0x02;
		bModelSolved = data->LabeledMarkers[i].params & 0x04;
		sMarker _marker = data->LabeledMarkers[i];
		marker.x = _marker.x;
		marker.y = _marker.y;
		marker.z = _marker.z;
		marker.oldName.markerIndex = _marker.ID;
		marker.id = markerIndex++;
		currentNetworkClient->allmarkerList[marker.id] = marker;

#if DEBUG_PRINT
		printf("Labeled Marker [ID=%d, Occluded=%d, PCSolved=%d, ModelSolved=%d] [size=%3.2f] [pos=%3.2f,%3.2f,%3.2f]\n",
			marker.ID, bOccluded, bPCSolved, bModelSolved, marker.size, marker.x, marker.y, marker.z);
#endif
	}
	for (i = 0; i < data->nOtherMarkers; i++)
	{
		//
		//printf("j:%d ", i);
		//printf("Oter Marker %f,%f%f\n", data->OtherMarkers[i][0], data->OtherMarkers[i][1], data->OtherMarkers[i][2]);
		Marker marker;


		marker.x = data->OtherMarkers[i][0];
		marker.y = data->OtherMarkers[i][1];
		marker.z = data->OtherMarkers[i][2];
		marker.id = markerIndex++;
		currentNetworkClient->allmarkerList[marker.id] = marker;
	}
	if (currentNetworkClient->allmarkerList.size()>0)
		currentNetworkClient->allMarkersCallback(currentNetworkClient->allmarkerList, frameCounter);

}

// MessageHandler receives NatNet error/debug messages
void __cdecl MessageHandler(int msgType, char* msg)
{
#if DEBUG_PRINT
	printf("\n%s\n", msg);
#endif
	printf("\n%s\n", msg);
}

// Establish a NatNet Client connection
int NetworkClient::CreateClient(char hostIP[128], char clientIP[128], int dataPort, int commandPort)
{
	// create NatNet client
	client = new NatNetClient(ConnectionType_Multicast);

	// [optional] use old multicast group
	//client->SetMulticastAddress("224.0.0.1");

	// print version info
	unsigned char ver[4];
	client->NatNetVersion(ver);
	printf("NatNet Sample Client (NatNet ver. %d.%d.%d.%d)\n", ver[0], ver[1], ver[2], ver[3]);

	// Set callback handlers
	client->SetMessageCallback(MessageHandler);
	client->SetVerbosityLevel(Verbosity_Error);// Verbosity_Debug);
	client->SetDataCallback(DataHandler, client);	// this function will receive data from the server

	myTimer.start();
	int retCode;
	int count = 0;
	bool problem;
	do {
		try {
			problem = false;
			// Init Client and connect to NatNet server
			// to use NatNet default port assigments
			retCode = client->Initialize(clientIP, hostIP);
		}
		catch (std::exception& e)
		{
			problem = true;
			printf("%s\n", e.what());
		}
	} while (problem && count++ < 10);

	// to use a different port for commands and/or data:
	//int retCode = client->Initialize(szMyIPAddress, szServerIPAddress, MyServersCommandPort, MyServersDataPort);
	if (retCode != ErrorCode_OK)
	{
		printf("Unable to connect to server.  Error code: %d. Exiting", retCode);
		return retCode;
	}
	else
	{
		// print server info
		sServerDescription ServerDescription;
		memset(&ServerDescription, 0, sizeof(ServerDescription));
		client->GetServerDescription(&ServerDescription);
		if (!ServerDescription.HostPresent)
		{
			printf("Unable to connect to server. Host not present. Exiting.");
			return 1;
		}
		printf("[SampleClient] Server application info:\n");
		printf("Application: %s (ver. %d.%d.%d.%d)\n", ServerDescription.szHostApp, ServerDescription.HostAppVersion[0],
			ServerDescription.HostAppVersion[1], ServerDescription.HostAppVersion[2], ServerDescription.HostAppVersion[3]);
		printf("NatNet Version: %d.%d.%d.%d\n", ServerDescription.NatNetVersion[0], ServerDescription.NatNetVersion[1],
			ServerDescription.NatNetVersion[2], ServerDescription.NatNetVersion[3]);
		printf("Client IP:%s\n", clientIP);
		printf("Server IP:%s\n", hostIP);
		printf("Server Name:%s\n\n", ServerDescription.szHostComputerName);
	}

	return ErrorCode_OK;
}
void NetworkClient::work(NetworkClient *netClient, char hostIP[128], char clientIP[128], int dataPort, int commandPort, bool unicast)
{
	netClient->waitMe = true;
	netClient->life = true;
	int iResult;
	int iConnectionType = ConnectionType_Multicast;
	if (unicast)
		iConnectionType = ConnectionType_Unicast;

	// Create NatNet Client
	iResult = netClient->CreateClient(hostIP, clientIP, dataPort, commandPort);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");

	}
	else
	{
		printf("Client initialized and ready.\n");
	}
	netClient->waitMe = false;
	while (netClient->life)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}
void NetworkClient::Initialize(char hostIP[128], char clientIP[128], int dataPort, int commandPort, bool unicast)
{
	//waitMe = false;
	//std::thread t (work, this, hostIP, clientIP, dataPort, commandPort, unicast);
	//t_id = t.get_id();
	//t.join();
	//printf("hello world\n");

	//while (waitMe)
	//	_sleep(100);// std::this_thread::sleep_for(std::chrono::milliseconds(1000));


	printf("HostIP: %s\n", hostIP);
	int iResult;
	int iConnectionType = ConnectionType_Multicast;
	if (unicast)
		iConnectionType = ConnectionType_Unicast;

	// Create NatNet Client
	iResult = CreateClient(hostIP, clientIP, dataPort, commandPort);
	if (iResult != ErrorCode_OK)
	{
		printf("Error initializing client.  See log for details.  Exiting");

	}
	else
	{
		printf("Client initialized and ready.\n");
	}


	// send/receive test request
	printf("[SampleClient] Sending Test Request\n");
	void* response;
	int nBytes;
	iResult = client->SendMessageAndWait("TestRequest", &response, &nBytes);
	if (iResult == ErrorCode_OK)
	{
		printf("[SampleClient] Received: %s", (char*)response);
	}

	//// Retrieve Data Descriptions from server
	//printf("\n\n[SampleClient] Requesting Data Descriptions...");
	//sDataDescriptions* pDataDefs = NULL;
	//int nBodies = client->GetDataDescriptions(&pDataDefs);
	//if (!pDataDefs)
	//{
	//	printf("[SampleClient] Unable to retrieve Data Descriptions.");
	//}
	//else
	//{
	//	printf("[SampleClient] Received %d Data Descriptions:\n", pDataDefs->nDataDescriptions);
	//	for (int i = 0; i < pDataDefs->nDataDescriptions; i++)
	//	{
	//		printf("Data Description # %d (type=%d)\n", i, pDataDefs->arrDataDescriptions[i].type);
	//		if (pDataDefs->arrDataDescriptions[i].type == Descriptor_MarkerSet)
	//		{
	//			// MarkerSet
	//			sMarkerSetDescription* pMS = pDataDefs->arrDataDescriptions[i].Data.MarkerSetDescription;
	//			printf("MarkerSet Name : %s\n", pMS->szName);
	//			for (int i = 0; i < pMS->nMarkers; i++)
	//				printf("%s\n", pMS->szMarkerNames[i]);

	//		}
	//		else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_RigidBody)
	//		{
	//			// RigidBody
	//			sRigidBodyDescription* pRB = pDataDefs->arrDataDescriptions[i].Data.RigidBodyDescription;
	//			printf("RigidBody Name : %s\n", pRB->szName);
	//			printf("RigidBody ID : %d\n", pRB->ID);
	//			printf("RigidBody Parent ID : %d\n", pRB->parentID);
	//			printf("Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
	//		}
	//		else if (pDataDefs->arrDataDescriptions[i].type == Descriptor_Skeleton)
	//		{
	//			// Skeleton
	//			sSkeletonDescription* pSK = pDataDefs->arrDataDescriptions[i].Data.SkeletonDescription;
	//			printf("Skeleton Name : %s\n", pSK->szName);
	//			printf("Skeleton ID : %d\n", pSK->skeletonID);
	//			printf("RigidBody (Bone) Count : %d\n", pSK->nRigidBodies);
	//			for (int j = 0; j < pSK->nRigidBodies; j++)
	//			{
	//				sRigidBodyDescription* pRB = &pSK->RigidBodies[j];
	//				printf("  RigidBody Name : %s\n", pRB->szName);
	//				printf("  RigidBody ID : %d\n", pRB->ID);
	//				printf("  RigidBody Parent ID : %d\n", pRB->parentID);
	//				printf("  Parent Offset : %3.2f,%3.2f,%3.2f\n", pRB->offsetx, pRB->offsety, pRB->offsetz);
	//			}
	//		}
	//		else
	//		{
	//			printf("Unknown data type.");
	//			// Unknown
	//		}
	//	}
	//}

	printf("\nClient is connected to server and listening for data...\n");
}

NetworkClient::NetworkClient(void(*rigidBodyCallback)(int index, int id, float *position, float *qrot, long frameCounter), void(*markersCallback)(MarkersList list, long frameCounter), void(*allMarkersCallback)(MarkersList list, long frameCounter))
{
	this->rigidBodyCallback = rigidBodyCallback;
	this->markersCallback = markersCallback;
	this->allMarkersCallback = allMarkersCallback;
	currentNetworkClient = this;
	//frameCounter = 0;
}

NetworkClient::~NetworkClient()
{
	life = false;
	client->Uninitialize();
	delete client;
}