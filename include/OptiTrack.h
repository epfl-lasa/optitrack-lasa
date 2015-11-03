
#ifndef _OPTITRACK_H_
#define _OPTITRACK_H_

// Select options
#define RBODY_FLIP_Y_Z
#define LHANDED_CORRECT


#define NAMEMAXLENGTH 1025
#define RBODYMAXMARKERS 10
#define MSETMAXMARKERS 15
#define MSETMAXNUMBERS 10
#define MSETMAXNO 10
#define OTHERMARKERMAXNO 10
#define MAXRBODY 10				//maximum no. of rigid bodies
#define SBODYMAXNO 30			//maximum no. of rigid bodies in a skeleton
#define MAXSKELETON 10			//maximum no. of skeletons
#define GETIDMAX 10

// NATNET message ids
#define NAT_PING                    0
#define NAT_PINGRESPONSE            1
#define NAT_REQUEST                 2
#define NAT_RESPONSE                3
#define NAT_REQUEST_MODELDEF        4
#define NAT_MODELDEF                5
#define NAT_REQUEST_FRAMEOFDATA     6
#define NAT_FRAMEOFDATA             7
#define NAT_MESSAGESTRING           8
#define NAT_UNRECOGNIZED_REQUEST    100
#define UNDEFINED                   999999.9999
#define SOCKET_ERROR 				-1

#define CPORT 1510						//command port
#define DPORT 1511						//data port

#define INVALID_POS_TOL 0.001

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <deque>
#include <string>
#include <string.h>
#include <fcntl.h>
#include <sys/select.h>
#include <boost/thread.hpp>
#include <boost/date_time.hpp>
#include <boost/thread/locks.hpp>
#include "Calibrator.h"

using namespace std;


// sender
typedef struct
{
    char szName[NAMEMAXLENGTH];            // sending app's name
    unsigned char Version[4];               // sending app's version [major.minor.build.revision]
    unsigned char NatNetVersion[4];         // sending app's NatNet version [major.minor.build.revision]

} sSender;

typedef struct
{
    unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
    unsigned short nDataBytes;              // Num bytes in payload
    union
    {
        unsigned char  cData[20000];
        char           szData[20000];
        unsigned long  lData[5000];
        float          fData[5000];
        sSender        Sender;
    } Data;                                 // Payload

} sPacket;


struct Marker
{
	int ID;
	float x;
	float y;
	float z;

	float fMarkerSize;
	int nMarkerID;
	char szMarkerName[NAMEMAXLENGTH];
	void print();
};

struct MarkerSet
{
	int nSetMarkers;	//assuming max no. of Marker Sets to be 10.
	char szMSetName[NAMEMAXLENGTH];
	struct Marker MSMarkers[MSETMAXMARKERS];	//assuming max. no. of markers in a marker set to be 15
	bool set;



	void print();

};

struct Rigid
{
	int ID;
	int parentID;
	float offsetX;
	float offsetY;
	float offsetZ;
	float x;
	float y;
	float z;
	float qx;
	float qy;
	float qz;
	float qw;
	char szRBodyName[NAMEMAXLENGTH];
	int nRigidMarkers;			//no. of markers on each rigid body
	struct Marker RMarkers[RBODYMAXMARKERS];	//assuming max. no. of Rigid body markers to be 10 per body
	float fError;				//mean marker error
	bool bMarkers;
	bool set;



	void print();
};

struct Skeleton
{
	int ID;
	int nRigidBodies;
	char szSkeletonName[NAMEMAXLENGTH];
	struct Rigid RBodies[SBODYMAXNO];
	bool bMarkers;
	bool set;


	void print();
};

struct SPosture
{
	double pos[3];
	double orient[3][3];
	double markerpos[5][3];
};

//if msg id = 7

class OptiTrack
{

private:


	fd_set mFdset;

	double tmpRotMat[3][3];
	double tmpTrans[3];
	char local_ip_addr[1025];
	volatile char pData[10000];
	volatile bool bStart;
	volatile bool bStopRequested;

	double rotmatrix[3][3];		//transformation matrices for
	double transmatrix[3];		//changing from world to robot frame

	char *ptr;
	volatile int sd;						//data socket
	int sc;						//command socket

	struct sockaddr_in localSock;
	struct ip_mreq group;

	struct sockaddr_in my_addr;
	struct sockaddr_in HostAddr;
	unsigned int ivalue;
	bool bFlag;
	int nlengthofsztemp;
	in_addr MyIPAddress;
	int nTries;
	bool bReceive;
	bool bCalibrate;

	boost::thread *mThread;
	boost::mutex *mutex;



	//int getRID[GETIDMAX];
	deque<int> getRID;
	deque<bool> bRDetails;
	deque <string> sRNames;

	//int getSID[GETIDMAX];
	deque<int> getSID;
	deque<bool> bSDetails;

	deque<int> RigidIDs;
	deque<long int>Rigidpos;
	deque<string> Rigidname;
//	char Rigidname[MAXRBODY][NAMEMAXLENGTH];

	deque<int> skeletonIDs;
	deque<long int> skeletonpos;
	deque<string> skeletonname;
//	char skeletonname[MAXRBODY][NAMEMAXLENGTH];
	char skrigidname[MAXSKELETON][MAXRBODY][NAMEMAXLENGTH];

	deque<long int> MarkerSetpos;
	char markersetname[NAMEMAXLENGTH][20];

	int nDataBytes;
	char getMSname[NAMEMAXLENGTH][GETIDMAX];
	char temp[NAMEMAXLENGTH];

	int ntemp, ntemp2, ntemp3;

	long int currentpos;

	bool bReadOtherMarks;

	bool bfirstrun;


    bool bPrintWarnings;

	int MessageID;
	int nDatasets;
	int type;
	int nBytes;
	int frameNumber;

	int nMarkerSets;
//	bool bMarkerDetails[MSETMAXNUMBERS];

	int nOtherMarkers;
	struct Marker OtherMarks[OTHERMARKERMAXNO];	//Coordinates of unidentified markers

	int nRigidBodies;
//	struct Rigid rBody[RBODYMAXNO];			//assuming max no. of rigid bodies to be 10
	deque<Rigid> rBody;

//	bool bRDetails[RBODYMAXNO];

	int nSkeletons;
//	struct Skeleton skeletons[SKELETONMAXNO];	//assuming max. no. of skeletons to be 10
	deque<Skeleton> skeletons;
	SPosture *skeletonPosture;
//	bool bSDetails[SKELETONMAXNO];

	float latency;
	long int latencypos;

	bool bMarkers;					//whether to print marker details

	int MSCount;
	int RBCount;
	int SKCount;


	void MatrixFromQuat(double qx, double qy, double qz, double qw, double rotmat[][3]);
	void CalibrateOrient(double orient[][3]);

	bool SocketOpen(const char* local_ip);
	bool SocketClose();

	bool commandsocket();
	void CalibratePos(double* position);

	bool receivedata();
	void UpdateFunc();

	volatile bool isOpened;

public:

	bool isInit;
	bool bUseThread;
//	struct MarkerSet markerSets[MSETMAXNUMBERS];		//assuming max. no. of marker sets to be 10
	deque<MarkerSet> markerSets;


	OptiTrack();
	~OptiTrack();

	int Init(const char* local_ip, bool useThread = false);

	void Start(){ bStart = true;}
	void Stop(){ bStart = false;}

	void DisplayInfo();			//display the rigid body names and ids present in the data

	bool enableRBody(int rid, bool details); 	//to set the rigid body IDs whose details are to be read
	bool enableRBody(const char *rbname, bool details);

	bool disableRBody(int rid);
	bool disableRBody(const char* name);

	bool enableSkeleton(int sid, bool details)	;	//to set the skeleton IDs whose details are to be read
	int enableSkeleton(const char *skname, bool details);

	bool disableSkeleton(int sid);

	void enableMSet(const char* msname);		//to set the skeleton IDs whose details are to be read

	void disableMSet(const char* msname);

	bool getRBodyPosition(double* position, int rid, float *timestamp=NULL);
	bool getRBodyPosition(double* position, const char* name, float *timestamp=NULL) ;

	bool getRBodyOrientation(double orient[][3], int rid, float *timestamp=NULL);
	bool getRBodyOrientation(double orient[][3], const char* name, float *timestamp=NULL) ;

	bool getRBodyOrientation(double **orient, int rid, float *timestamp=NULL);
	bool getRBodyOrientation(double **orient, const char* name, float *timestamp=NULL) ;


	vector<string> GetRBodyNameList();
	vector<int> GetRBodyIDList();

	bool RBidtoname(int &rid, char *rbname);
	bool RBnametoid(int &rid, char *rbname);


	vector<string> GetSkeletonNameList();

	bool getRBody(Rigid &rbodytemp, int rid, float* timestamp=NULL);
	bool getSkeleton(Skeleton &pskeleton, int sid );
	bool getSkeleton(SPosture *pskeleton, int sid );
	bool getMarkerSet(MarkerSet &markersettemp, char *setname);

	int Update();
	int Refresh();

	bool loadCalibrationMatrix(const char *filename);
	bool loadCalibrationMatrix(double *rotation[3], double *translate);

    void enableWarnings(bool bb);
    int getFrameNumber();

};

#endif

