#include "VisionPkg/OptiTrack.h"
#include <iostream>

#define NORM_TOLERANCE 0.01
#define UNIT 1

using namespace std;


struct MarkerSet markertemp;
struct Rigid rigidtemp;
struct Skeleton skeletontemp;

float xtemp, ytemp, ztemp;

void Marker::print()
{
	cout<<endl<<"Marker ID: "<<ID
			<<endl<<"Marker Name:"<<szMarkerName
			<<endl<<"Marker Size: "<<fMarkerSize
			<<endl<<"Position: "<<x<<"\t"<<y<<"\t"<<z<<endl;
}

void MarkerSet::print()
{
	cout<<endl<<"Markerset\n no of markers: "<<nSetMarkers
			<<endl<<"MarkerSet Name:"<<szMSetName;

	for(int i =0; i <nSetMarkers; i++)
		MSMarkers[i].print();
}

void Rigid::print()
{
	if(set)
	{
		//		cout<<"\nBool:"<<set<<endl;
		cout<<endl<<"Rigid Body ID:\t"<<ID<<endl
				<<"Name:\t"<<szRBodyName<<endl
				<<"position:\t"<<x<<"\t"<<y<<" \t"<<z<<endl
				<<"orientation:\t"<<qx<<"\t"<<qy<<"\t"<<qz<<"\t"<<qw<<endl
				<<"No. of Rigid Body Markers:\t"<<nRigidMarkers<<endl;
		if(bMarkers)
		{
			cout<<endl<<"Marker Details:\n";
			for(int i = 0; i<nRigidMarkers; i++)
			{
				RMarkers[i].print();
			}
			cout<<endl<<"Mean Marker Error: "<<fError<<endl;
		}
	}
	else
		cout<<"\nRigid body details can't be printed"<<endl;
}

void Skeleton::print()
{
	cout<<"\nBool:"<<set<<endl;
	if(set)
	{
		cout<<endl<<"Skeleton ID:\t"<<ID<<endl
				<<"Name:\t"<<szSkeletonName<<endl
				<<"No. of  Rigid Bodies:\t"<<nRigidBodies<<endl
				<<"List & Details of Rigid Bodies:"<<endl;
		for(int i =0; i<nRigidBodies; i++)
		{
			RBodies[i].print();
		}
	}
	else
		cout<<"\nSkeleton details can't be printed "<<endl;
}

OptiTrack::OptiTrack()
{
	isOpened = false;
	isInit = false;
	bReceive = true;
	MSCount = 0;
	RBCount = 0;
	SKCount = 0;

	bStopRequested = false;
	bStart  = false;
	bCalibrate = false;

	bPrintWarnings = true;
}

OptiTrack::~OptiTrack()
{

	if(bUseThread && isInit)
	{
		bStopRequested = true;
		mThread->join();
	}

	usleep(100000);
	close(sd);

}


bool OptiTrack::loadCalibrationMatrix(const char *filename)
{
	int dummy;
	FILE *calib= fopen(filename, "r+");
	double norm = 0.0;
	if(!calib)
	{
		bCalibrate = false;
		cout<<"ERROR: Calibration file not found!!"<<endl;
		return false;
	}

	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
		{
			dummy = fscanf(calib,"%lf", &(rotmatrix[i][j]));
		}
	}

	for(int j=0; j<3; j++)
	{
		for(int i=0; i<3; i++)
		{
			norm += rotmatrix[i][j]*rotmatrix[i][j];
		}
		if(norm<UNIT-NORM_TOLERANCE || norm>UNIT+NORM_TOLERANCE)
		{
			cout<<"Rotation Matrix Not normalized or Invalid.Calibration not loaded."<<endl;
			bCalibrate = false;
			return false;

		}
		norm = 0.0;
	}

	for(int i=0; i<3; i++)
		dummy = fscanf(calib, "%lf", &(transmatrix[i]));
	fclose(calib);
	cout<<"Calibration file: "<<filename<<" loaded!"<<endl;
	cout<<"Rotation :"<<endl;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
			cout<<rotmatrix[i][j]<<" \t";
		cout<<endl;
	}

	bCalibrate = true;
	return true;
}


bool OptiTrack::loadCalibrationMatrix(double *rotation[3], double *translate)
{
	double norm;
	norm =0.0;
	for(int j=0; j<3; j++)
	{
		for(int i=0; i<3; i++)
		{
			rotmatrix[i][j] = rotation[i][j];
			norm += rotmatrix[i][j]*rotmatrix[i][j];
		}
		if(norm<UNIT-NORM_TOLERANCE || norm>UNIT+NORM_TOLERANCE)
		{
			cout<<"Rotation Matrix Not normalized or Invalid.\nRotation matrix not loaded."<<endl;
			bCalibrate = false;
			return false;

		}
		transmatrix[j] = translate[j];
		norm = 0.0;
	}

	cout<<"Rotation :"<<endl;
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<3; j++)
			cout<<rotmatrix[i][j]<<" \t";
		cout<<endl;
	}

	bCalibrate = true;
	return true;
}

void OptiTrack::enableWarnings(bool bb)
{
	bPrintWarnings = bb;
}

int OptiTrack::getFrameNumber()
{
	return frameNumber;
}

void OptiTrack::CalibratePos(double* position)
{

	double x = position[0];
	double y = position[1];
	double z = position[2];

	position[0] = rotmatrix[0][0]*x + rotmatrix[0][1]*y + rotmatrix[0][2]*z + transmatrix[0];
	position[1] = rotmatrix[1][0]*x + rotmatrix[1][1]*y + rotmatrix[1][2]*z + transmatrix[1];
	position[2] = rotmatrix[2][0]*x + rotmatrix[2][1]*y + rotmatrix[2][2]*z + transmatrix[2];

	//	cout<<"\nCalibrated Positions:\t"<<xtemp<<"\t"<<ytemp<<"\t"<<ztemp<<endl;
}

void OptiTrack::CalibrateOrient(double orient[][3])
{

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
		{
			tmpRotMat[i][j] = 0;

			for(int k=0;k<3;k++)
				tmpRotMat[i][j] += rotmatrix[i][k]*orient[k][j];
		}

	for(int i=0;i<3;i++)
		for(int j=0;j<3;j++)
			orient[i][j] = tmpRotMat[i][j];

}

void OptiTrack::UpdateFunc()
{

	fd_set rdfs;
	FD_ZERO(&rdfs);
	FD_SET(sd, &rdfs);

	int read_result;
	while(!bStopRequested)
	{
		if(isOpened)
		{
			if(bStart)
			{
				struct timeval tm;
				tm.tv_sec = 1;
				tm.tv_usec = 000;
				read_result = select(sd+1, &rdfs, NULL, NULL, &tm);

				if(read_result == -1)
				{
					if(bPrintWarnings)
					{
						cout<<"ERROR while selecing socket"<<endl;
						cout<<"Stopping thread"<<endl;
					}
					bStart = false;
				}
				else if(read_result)
				{
					mutex->lock();
					read_result = receivedata();
					mutex->unlock();
				}
				else
				{
					bStart = false;
					if(bPrintWarnings)
						cout<<"Stopping thread"<<endl;
				}

			}
		}
		else
		{
			//			cout<<"Socket not openend!!"<<endl;
			continue;
		}

	}

	cout<<"Exiting"<<endl;

}
int OptiTrack::Init(const char* local_ip, bool useThread)
{
	bUseThread = useThread;
	cout<<"Initializing..."<<endl;
	memcpy(local_ip_addr, local_ip, sizeof(local_ip));
	if(!SocketOpen(local_ip))
	{
		cout<<"Cannot open socket with the specified ip. Check your local ip!!"<<endl;
		return -1;
	}
	usleep(100000);
	FD_ZERO(&mFdset);
	FD_SET(sd, &mFdset);
	struct timeval timeout;
	timeout.tv_sec = 1;
	timeout.tv_usec = 0000;

	int read_result = select(sd+1, &mFdset, NULL, NULL, &timeout);
	if(read_result == -1)
	{
		cout<<"Select failed on socket!"<<endl;
		return -2;
	}
	else if(!read_result)
	{
		cout<<"No data on socket!"<<endl;
		return -2;
	}

	cout<<"Trying to receive data...";
	if(!receivedata())
	{
		cout<<"Cannot receive data!!"<<endl;
		return -2;
	}
	Start();
	usleep(100000);

	if(Refresh()< 0)
	{
		cout<<"Cannot refresh"<<endl;
		return -2;
	}

	if(bUseThread)
	{
		cout<<"Creating thread...";
		mThread = new boost::thread(boost::bind(&OptiTrack::UpdateFunc, this));
		mutex = new boost::mutex();
	}
	cout<<"Initializing done"<<endl;
	isInit = true;

	return 1;

}

void OptiTrack::MatrixFromQuat(double qx, double qy, double qz, double qw, double rotmat[][3])
{
	rotmat[0][0] = qx*qx + qw*qw - qz*qz - qy*qy ;
	rotmat[0][1] = 2*(qx*qy - qz*qw);
	rotmat[0][2] = 2*(qx*qz + qy*qw);
	rotmat[1][0] = 2*(qx*qy + qz*qw);
	rotmat[1][1] = qy*qy - qx*qx + qw*qw - qz*qz ;
	rotmat[1][2] = 2*(qy*qz - qx*qw);
	rotmat[2][0] = 2*(qx*qz - qy*qw);
	rotmat[2][1] = 2*(qy*qz + qx*qw);
	rotmat[2][2] = qz*qz- qy*qy - qx*qx + qw*qw ;


}

bool OptiTrack::RBidtoname(int &rid, char *rbname)
{
	for(unsigned int i=0; i<RigidIDs.size(); i++)
	{
		if( rid == RigidIDs[i])
		{
			strcpy(rbname, Rigidname[i].c_str());
			return true;
		}

	}
	return false;
}

bool OptiTrack::RBnametoid(int &rid, char *rbname)
{
	for(long int i=0; i<(long)RigidIDs.size(); i++)
	{
		if( strcmp(rbname, Rigidname[i].c_str()))
		{
			rid = RigidIDs[i];
			return true;
		}

	}
	return false;
}


vector<int> OptiTrack::GetRBodyIDList()
{
	vector<int> rid;
	for(long int i=0; i<(long)RigidIDs.size(); i++)
		rid.push_back(RigidIDs[i]);
	return rid;
}

vector<string> OptiTrack::GetRBodyNameList()
{
	vector<string> rbnames;
	for(long int i =0; i< (long) RigidIDs.size(); i++)
		rbnames.push_back(Rigidname[i]);


	return rbnames;
}

vector<string> OptiTrack::GetSkeletonNameList()
{
	vector<string> rbnames;

	for(long int i =0; i< (long) skeletonIDs.size(); i++)
		rbnames.push_back(skeletonname[i]);

	return rbnames;
}

void OptiTrack::DisplayInfo()
{
	if(RigidIDs.size()>0)
	{
		cout<<"Displaying list of rigid bodies...\nID \t \t Name "<<endl;
		for(long int i =0; i<(long)RigidIDs.size(); i++)
		{
			cout<<RigidIDs[i]<<"\t \t "<<Rigidname[i]<<endl;
		}
	}
	if(skeletonIDs.size()>0)
	{
		cout<<"Displaying list of skeletons...\nID \t \t Name "<<endl;
		for(long int i =0; i<(long)RigidIDs.size(); i++)
		{
			cout<<skeletonIDs[i]<<"\t \t "<<skeletonname[i]<<endl;
		}

	}
}

bool OptiTrack::enableRBody(int rid, bool details)
{
	for(long int i=0; i<(long)RigidIDs.size(); i++)
	{
		if(RigidIDs[i] == rid)
		{
			getRID.push_back(rid);
			bRDetails.push_back(details);

			//TODO: check here validity of Rigidname[i]
			sRNames.push_back(Rigidname[i]);
			if(bPrintWarnings)
				cout<<"Enabled Rigid Body "<<rid<<endl;
			return true;

		}
	}

	if(bPrintWarnings)
		cout<<"Rigid Body ID: "<<rid<<" not found in the data!"<<endl;
	return false;

}

bool OptiTrack::enableRBody(const char *rbname, bool details)
{


	for(long int i=0; i<(long)Rigidname.size(); i++)
	{
		if(strcmp(Rigidname[i].c_str(), rbname) == 0)
		{

			getRID.push_back(RigidIDs[i]);		//adding the rigid ID corresponding to rigid name
			bRDetails.push_back(details);
			sRNames.push_back(rbname);
			if(bPrintWarnings)
				cout<<"Enabled Rigid Body name: "<<rbname<<" ID: "<<RigidIDs[i]<<endl;
			return true;

		}
	}
	if(bPrintWarnings)
		cout<<"Rigid Body name: "<<rbname<<" not found in the data!"<<endl;
	return false;
}

bool OptiTrack::disableRBody(int rid)
{
	if(getRID.size()>0)
	{
		for(long i =0; i<(long)getRID.size(); i++)
		{
			if(rid == getRID.at(i))
			{
				if(bPrintWarnings)
					cout<<"Rigid Body ID: "<<getRID.at(i)<<" disabled"<<endl;
				getRID.erase(getRID.begin()+i);
				bRDetails.erase(bRDetails.begin()+i);
				sRNames.erase(sRNames.begin()+i);
				return true;
			}
		}
		if(bPrintWarnings)
			cout<<"Rigid Body ID: "<<rid<<" not enabled yet!"<<endl;
		return false;

	}
	else
		cout<<"No Rigid body activated yet"<<endl;
	return false;
}

bool OptiTrack::disableRBody(const char* name)
{
	if(getRID.size()>0)
	{
		for(long i =0; i<(long)getRID.size(); i++)
		{
			if(strcmp(sRNames[i].c_str(), name) == 0)
			{
				getRID.erase(getRID.begin()+i);
				bRDetails.erase(bRDetails.begin()+i);
				sRNames.erase(sRNames.begin()+i);
				if(bPrintWarnings)
					cout<<"Rigid Body : "<<name<<" disabled"<<endl;
				return true;
			}
		}

		if(bPrintWarnings)
			cout<<"Rigid Body : "<<name<<" not enabled yet!"<<endl;
		return false;

	}
	else
		cout<<"No Rigid body activated yet"<<endl;
	return false;
}

bool OptiTrack::enableSkeleton(int sid, bool details)
{
	for(long int i = 0; i<(long)getSID.size(); i++)
	{
		if(sid == getSID[i])
		{
			getSID.push_back(sid);
			bSDetails.push_back(details);
			cout<<"Enabled Skeleton ID "<<sid<<endl;
			return true;
		}
	}

	cout<<"Skeleton ID: "<<sid<<" not found in the data!"<<endl;
	return false;
}


int OptiTrack::enableSkeleton(const char* skname, bool details)
{
	for(long int i=0; i<(long)skeletonname.size(); i++)
	{
		cout << "skeleton name: " << skeletonname[i] << endl;
		if(strcmp(skeletonname[i].c_str(), skname) == 0)
		{
			getSID.push_back(skeletonIDs[i]);		//adding the rigid ID corresponding to rigid name
			bSDetails.push_back(details);
			cout<<"Enabled Skeleton name: "<<skname<<" ID: "<<skeletonIDs[i]<<endl;
			return skeletonIDs[i];

		}
	}

	cout<<"Skeleton name: "<<skname<<" not found in the data!"<<endl;
	return -1;
}
bool OptiTrack::disableSkeleton(int sid)
{
	//		getSID[sid] = 0;
	//		bSDetails[sid] = false;
	if(getSID.size()>0)
	{
		for(long i =0; i<(long)getSID.size(); i++)
		{
			if(sid == getSID.at(i))
			{
				cout<<"\nSkeleton ID: "<<getSID.at(i)<<" disabled"<<endl;
				getSID.erase(getSID.begin()+i);
				bSDetails.erase(bSDetails.begin()+i);
				return true;
			}
		}
		cout<<"Skeleton ID: "<<sid<<" not enabled yet"<<endl;
		return false;

	}
	else
		cout<<"No Skeleton activated yet"<<endl;
	return false;
}


bool OptiTrack::getRBodyPosition(double* position, int rid, float *timestamp) // getRBodyPose(Vector &pose, int rid, float *timestamp) { pose[0] = rBody[i].x; ... }
{

	long int i = 0;
	while(i<(long)getRID.size())
	{
		if(rBody[i].ID == rid)
		{

			if(fabs(rBody[i].x) < INVALID_POS_TOL && fabs(rBody[i].y) < INVALID_POS_TOL && fabs(rBody[i].z) < INVALID_POS_TOL){
				if(bPrintWarnings)
					cout<<"WARNING: Invalid position [0 0 0] detected"<<endl;
				return false;
			}
			position[0] = rBody[i].x;
			position[1] = rBody[i].z;
			position[2] = rBody[i].y;

			if(bCalibrate)
				CalibratePos(position);

			if(timestamp != NULL)
				(*timestamp) = latency;
			//			mutex->unlock();
			return true;
		}
		else
			i++;
	}
	if(bPrintWarnings)
		cout<<"Rigid Body ID: "<<rid<<" not found"<<endl;

	return false;

}

bool OptiTrack::getRBodyOrientation(double orient[][3], int rid, float *timestamp) // getRBodyPose(Vector &pose, int rid, float *timestamp) { pose[0] = rBody[i].x; ... }
{

	long int i = 0;
	while(i<(long)getRID.size())
	{
		if(rBody[i].ID == rid)
		{

			MatrixFromQuat(rBody[i].qx, rBody[i].qy, rBody[i].qz, rBody[i].qw, orient);

			if(bCalibrate)
				CalibrateOrient(orient);

			if(timestamp != NULL)
				(*timestamp) = latency;
			//			mutex->unlock();
			return true;
		}
		else
			i++;
	}
	if(bPrintWarnings)
		cout<<"Rigid Body ID: "<<rid<<" not found"<<endl;

	return false;

}

bool OptiTrack::getRBodyOrientation(double **orient, int rid, float *timestamp)
{
	double lOrient[3][3];
    bool b_opti = getRBodyOrientation(lOrient, rid, timestamp);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            orient[i][j] = lOrient[i][j];
    return b_opti;
}

bool OptiTrack::getRBodyOrientation(double **orient, const char* name, float *timestamp)
{
	double lOrient[3][3];
    bool b_opti = getRBodyOrientation(lOrient, name, timestamp);
    for(int i=0; i<3; i++)
        for(int j=0; j<3; j++)
            orient[i][j] = lOrient[i][j];
    return b_opti;
}


bool OptiTrack::getRBodyPosition(double* position, const char* name, float *timestamp) // getRBodyPose(Vector &pose, int rid, float *timestamp) { pose[0] = rBody[i].x; ... }
{
	long int i = 0;

	while(i<(long)getRID.size())
	{
		if(strcmp(rBody[i].szRBodyName, name) == 0)
		{

			if(fabs(rBody[i].x) < INVALID_POS_TOL && fabs(rBody[i].y) < INVALID_POS_TOL && fabs(rBody[i].z) < INVALID_POS_TOL){
				if(bPrintWarnings)
					cout<<"WARNING: Invalid position [0 0 0] detected"<<endl;
				return false;
			}

			position[0] = rBody[i].x;
			position[1] = rBody[i].z;
			position[2] = rBody[i].y;

			if(bCalibrate)
				CalibratePos(position);
			if(timestamp != NULL)
				(*timestamp) = latency;

			//			mutex->unlock();
			return true;
		}
		else
			i++;
	}

	if(bPrintWarnings)
		cout<<"Rigid Body : "<<name<<" not found"<<endl;

	return false;
}


bool OptiTrack::getRBodyOrientation(double orient[][3], const char* name, float *timestamp) // getRBodyPose(Vector &pose, int rid, float *timestamp) { pose[0] = rBody[i].x; ... }
{
	long int i = 0;

	while(i<(long)getRID.size())
	{
		if(strcmp(rBody[i].szRBodyName, name) == 0)
		{

			MatrixFromQuat(rBody[i].qx, rBody[i].qy, rBody[i].qz, rBody[i].qw, orient);

			if(bCalibrate)
				CalibrateOrient(orient);

			if(timestamp != NULL)
				(*timestamp) = latency;
			//			mutex->unlock();
			return true;
		}
		else
			i++;
	}
	if(bPrintWarnings)
		cout<<"Rigid Body : "<<name<<" not found"<<endl;

	return false;

}

bool OptiTrack::getRBody(Rigid &rbodytemp, int rid, float* timestamp) // getRBodyPose(Vector &pose, int rid, float *timestamp) { pose[0] = rBody[i].x; ... }
{
	long int i = 0;
	while(i<(long)getRID.size())
	{
		if(rBody[i].ID == rid)
		{
			rbodytemp = rBody[i];
			if(timestamp != NULL)
				(*timestamp) = latency;
			return true;
		}
		else
			i++;
	}

	cout<<"Rigid Body ID: "<<rid<<" not found"<<endl;

	return false;

}



bool OptiTrack::getSkeleton(Skeleton &pskeleton, int sid)
{
	long int i = 0;
	pskeleton.set = false;
	while(i<(long)getSID.size())
	{
		if(skeletons[i].ID == sid)
		{
			pskeleton = skeletons[i];
			return true;
		}
		else
			i++;
	}

	cout<<"Skeleton ID: "<<sid<<" not found"<<endl;
	return false;
}

bool OptiTrack::getSkeleton(SPosture *pskeleton, int sid )
{
	long int i = 0;
	double lPos[3];

	while(i<(long)getSID.size())
	{
		if(skeletons[i].ID == sid)
		{
			for(int j=0; j<skeletons[i].nRigidBodies; j++)
			{
				pskeleton[j].pos[0] = skeletons[i].RBodies[j].x;
				pskeleton[j].pos[1] = skeletons[i].RBodies[j].y;
				pskeleton[j].pos[2] = skeletons[i].RBodies[j].z;

				//CalibratePos(pskeleton[j].pos);

				MatrixFromQuat(skeletons[i].RBodies[j].qx,
						skeletons[i].RBodies[j].qy,
						skeletons[i].RBodies[j].qz,
						skeletons[i].RBodies[j].qw,
						pskeleton[j].orient );

				for(int k=0; k<skeletons[i].RBodies[j].nRigidMarkers; k++)
				{
					lPos[0] = skeletons[i].RBodies[j].RMarkers[k].x;
					lPos[1] = skeletons[i].RBodies[j].RMarkers[k].y;
					lPos[2] = skeletons[i].RBodies[j].RMarkers[k].z;

					if(bCalibrate)
					{
						CalibratePos(lPos);
					}

					pskeleton[j].markerpos[k][0] = lPos[0];
					pskeleton[j].markerpos[k][1] = lPos[1];
					pskeleton[j].markerpos[k][2] = lPos[2];

					if(k>4) break;
				}

				if(bCalibrate)
					CalibrateOrient(pskeleton[j].orient);
			}
			return true;
		}
		else
			i++;
	}

	cout<<"Skeleton ID: "<<sid<<" not found"<<endl;
	return false;
}

int OptiTrack::Refresh()
{
	cout<<"Refreshing..."<<endl;
	MarkerSetpos.clear();
	markerSets.clear();
	RigidIDs.clear();
	Rigidpos.clear();
	rBody.clear();
	skeletonIDs.clear();
	skeletonpos.clear();
	skeletons.clear();

	Stop();
	usleep(100000);
	if(isOpened)
	{

		if(!receivedata())
		{
			cout<<"Cannot receive data!!"<<endl;
			return -1;
		}
		else
		{
			ptr = (char*)pData;
		}

	}
	else
	{
		cout<<"Socket not openend!!"<<endl;
		return -1;
	}
	Start();
	usleep(100000);


	long int ptrbegin = (long int) ptr;

	MessageID = 0;
	memcpy(&MessageID, ptr, 2);
	//	cout<<"\nMessageID : "<<MsgID;
	ptr += 2;
	nBytes = 0;
	memcpy(&nBytes, ptr, 2);
	//	cout<<"ByteCount: "<<nBytes<<endl;
	ptr += 2;


	if(MessageID == 7)      // FRAME OF MOCAP DATA packet
	{
		// frame number
		memcpy(&frameNumber, ptr, 4);
		ptr += 4;
		//		cout<<"Frame No. : "<<frameNumber<<endl;

		// number of data sets (markersets, rigidbodies, etc)
		memcpy(&nMarkerSets, ptr, 4);		//temp = number of marker sets
		ptr += 4;
		//		cout<<"MarkerSets:"<<nMarkerSets<<endl;

		for (int i=0; i < nMarkerSets; i++)
		{
			// Markerset name
			strcpy(temp, ptr);
			//			cout<<"temp:"<<temp<<endl;
			currentpos = (long)ptr - ptrbegin;
			MarkerSetpos.push_back(currentpos);
			markerSets.push_back(markertemp);


			nDataBytes = (int) strlen(temp) + 1;
			ptr += nDataBytes;

			// marker data
			memcpy(&ntemp2, ptr, 4);
			ptr += 4;

			ptr += 4*3*ntemp2;

		}

		// unidentified markers


		memcpy(&nOtherMarkers, ptr, 4);			//temp = number of other markers
		ptr += 4;
		ptr += 4*3*nOtherMarkers;
		// rigid bodies
		memcpy(&nRigidBodies, ptr, 4);		//ntemp = no. of rigid bodies
		ptr += 4;
		//		cout<<"read no. of rigid bodies "<<nRigidBodies<<endl;
		for (int j=0; j < nRigidBodies; j++)
		{
			// rigid body pos/ori
			memcpy(&ntemp2, ptr+2, 2);
			//			cout<<"\nRigid ID::: "<<ntemp2;
			currentpos = (long int)ptr - ptrbegin;
			ptr += 4;
			RigidIDs.push_back(ntemp2);

			Rigidpos.push_back(currentpos);
			rBody.push_back(rigidtemp);


			ptr+= 4*7;
			memcpy(&ntemp2, ptr, 4);		//ntemp2 = no. of rigid markers
			ptr += 4;
			ptr += 4 + 4*5*ntemp2;
			//		            	rBody[j].set = false;
			//		            }
		} // next rigid body


		// skeletons
		memcpy(&nSkeletons, ptr, 4); ptr += 4;	//ntemp = no. of skeletons

		for (int j=0; j < nSkeletons; j++)
		{
			// skeleton id
			currentpos = (long)ptr - ptrbegin;
			int skeletonID = 0;
			memcpy(&skeletonID, ptr, 4); ptr += 4;
			int nRigidBodies = 0;
			memcpy(&nRigidBodies, ptr, 4); ptr += 4;

			skeletonIDs.push_back(skeletonID);
			skeletonpos.push_back(currentpos);
			skeletons.push_back(skeletontemp);

			for (int i=0; i < nRigidBodies; i++)
			{
				ptr += 8*4;                            // rigid body pos/ori

                memcpy(&ntemp2, ptr, 4); ptr += 4;

                ptr += ntemp2*3*4;       // associated marker positions
                ptr += ntemp2*4;         // associated marker IDs
                ptr += ntemp2*4;         // associated marker sizes
                ptr += 4;                // mean marker error
			}
		} // next skeleton


		// latency
		latencypos = (long)ptr - ptrbegin;
		latency = 0.0f;
		memcpy(&latency, ptr, 4); ptr += 4;

        int eod = 0; memcpy(&eod, ptr, 4);

		//		cout<<"latency: "<<latency;

		//		memcpy(&eod, ptr, 4); ptr += 4; cout<<"eod1: "<<eod<<endl;

		//		cout<<"End Packet\n-------------"<<endl;
		//		long int diff = (long)ptr - ptrbegin;
		//		cout<<"\nTotal packet size = "<<diff<<endl;


		cout<<MarkerSetpos.size()<<" MarkerSets found."<<endl;
		cout<<RigidIDs.size()<<" Rigid Bodies found."<<endl;
		cout<<skeletonIDs.size()<<" Skeletons found."<<endl;
	}

	else
	{
		printf("Unrecognized Packet Type while refreshing frame of data.\n");
		return -1;
	}

	if(!commandsocket())
	{
		cout<<"Command socket failed!!"<<endl;
		return -1;
	}


	MessageID = 0;
	memcpy(&MessageID, ptr, 2);
	//	cout<<"\nMessageID : "<<MsgID;
	ptr += 2;
	nBytes = 0;
	memcpy(&nBytes, ptr, 2);
	//	cout<<"\nByteCount: "<<nBytes;
	ptr += 2;

	if(MessageID == 5)
	{
		bool bfound;

		nDatasets = 0;
		memcpy(&nDatasets, ptr, 4); ptr += 4;
		//		cout<<endl<<"Dataset Count : "<<nDatasets;
		for(int i=0; i < nDatasets; i++)
		{
			type = 0;
			memcpy(&type, ptr, 4); ptr += 4;
			//			cout<<"\nType:"<<type;

			if(type == 0)   // markerset
			{
				// name
				strcpy(markerSets[MSCount].szMSetName, ptr);
				nDataBytes = (int) strlen(markerSets[MSCount].szMSetName) + 1;
				ptr += nDataBytes;
				//				printf("Markerset Name: %s\n", markerSets[MSCount].szMSetName);

				// marker data
				markerSets[MSCount].nSetMarkers =0;

				memcpy(&markerSets[MSCount].nSetMarkers, ptr, 4); ptr += 4;
				//				printf("Marker Count : %d\n", nMarkers);

				for(int j=0; j < markerSets[MSCount].nSetMarkers; j++)
				{
					//					char szName[256];
					strcpy(markerSets[MSCount].MSMarkers[j].szMarkerName, ptr);
					nDataBytes = (int) strlen(markerSets[MSCount].MSMarkers[j].szMarkerName) + 1;
					ptr += nDataBytes;
					//					printf("Marker Name: %s\n", szName);
				}
				MSCount++;
			}
			else if(type ==1)   // rigid body
			{
				//				if(major >= 2)
				//				{
				// name
				//				char szName[MAX_NAMELENGTH];
				strcpy(temp, ptr);
				ptr += strlen(ptr) + 1;
				//				printf("Name: %s\n", szName);
				//				}

				//					rBody[RBCount].ID = 0;
				memcpy(&ntemp, ptr,2); ptr +=4;
				//				printf("ID : %d\n", ID);

				bfound = false;
				for(long int k=0; k<(long)RigidIDs.size();k++)
				{
					if(ntemp == RigidIDs[k])
					{
						//strcpy(Rigidname[k], temp);
						Rigidname.push_back(temp);
						//						cout<<endl<<"Rigid Name: "<<temp<<endl;
						bfound = true;
						break;
					}
				}

				if(!bfound)
				{
					cout<<"\nRigid Body ID: "<<ntemp<<" in data descriptor message is not present in frameofdata message!"
							<<"\nIgnoring Rigid Body ID: "<<ntemp<<" name:"<<temp<<endl;
					//return -1;
				}
				//					int parentID = 0; memcpy(&parentID, ptr, 4);
				ptr +=4;
				//					printf("Parent ID : %d\n", parentID);

				//					float xoffset = 0; memcpy(&xoffset, ptr, 4);
				ptr +=4;
				//					printf("X Offset : %3.2f\n", xoffset);

				//					float yoffset = 0; memcpy(&yoffset, ptr, 4);
				ptr +=4;
				//					printf("Y Offset : %3.2f\n", yoffset);

				//					float zoffset = 0; memcpy(&zoffset, ptr, 4);
				ptr +=4;
				//					printf("Z Offset : %3.2f\n", zoffset);
			}
			else if(type ==2)   // skeleton
			{
				strcpy(temp, ptr);
				ptr += strlen(ptr) + 1;
				//				printf("Name: %s\n", szName);

				//					skeletons[SKCount].ID = 0;
				memcpy(&ntemp, ptr, 4); ptr +=4;
				//				printf("ID : %d\n", ID);

				bfound = false;
				long int k;
				for(k=0; k<(long)skeletonIDs.size(); k++)
				{
					if(ntemp == skeletonIDs[k])
					{
						skeletonname.push_back(temp);
						bfound = true;
						break;
					}
				}
				if(!bfound)
				{
					cout<<"\nSkeleton ID: "<<ntemp<<" in data descriptor message is not present in frameofdata message!"
							<<"\nIgnoring Skeleton ID: "<<ntemp<<" name:"<<temp<<endl;
					return -1;
				}

				//arrangement of the names in skeletonname deque:

				//					skeletons[SKCount].nRigidBodies = 0;
				memcpy(&ntemp, ptr, 4);
				ptr +=4;
				//				printf("RigidBody (Bone) Count : %d\n", nRigidBodies);

				for(int l=0; l< ntemp; l++)		//ntemp = no. of rigid bodies in the skeleton
				{
					//					if(major >= 2)
					//					{
					// RB name
					//						char szName[MAX_NAMELENGTH];
					strcpy(temp, ptr);
					strcpy(skrigidname[k][l], temp);
					ptr += strlen(ptr) + 1;
					ptr += 4*5;
					//						printf("Rigid Body Name: %s\n", szName);
					//					}

					//						skeletons[SKCount].RBodies[k].ID = 0;
					//						memcpy(&skeletons[SKCount].RBodies[k].ID, ptr, 4);
					//						ptr +=4;
					//					printf("RigidBody ID : %d\n", ID);

					//						skeletons[SKCount].RBodies[k].parentID = 0;
					//						memcpy(&skeletons[SKCount].RBodies[k].parentID, ptr, 4);
					//						ptr +=4;
					//					printf("Parent ID : %d\n", parentID);

					//						skeletons[SKCount].RBodies[k].offsetX = 0;
					//						memcpy(&skeletons[SKCount].RBodies[k].offsetX, ptr, 4);
					//						ptr +=4;
					//					printf("X Offset : %3.2f\n", xoffset);

					//						skeletons[SKCount].RBodies[k].offsetY = 0;
					//						memcpy(&skeletons[SKCount].RBodies[k].offsetY, ptr, 4);
					//						ptr +=4;
					//					printf("Y Offset : %3.2f\n", yoffset);

					//						skeletons[SKCount].RBodies[k].offsetZ = 0;
					//						memcpy(&skeletons[SKCount].RBodies[k].offsetZ, ptr, 4);
					//						ptr +=4;
					//					printf("Z Offset : %3.2f\n", zoffset);
				}
			}

		}   // next dataset


		//		printf("End Packet\n-------------\n");
		return 1;
	}

	else
	{
		printf("Unrecognized Packet Type while refreshing model description.\n");
		return -1;
	}

	return 1;

}

int OptiTrack::Update()
{
	double lPos[3];

	if(bUseThread)
	{
		if(!bStart)
		{
			if(bPrintWarnings)
				cout<<"Thread not working!"<<endl;
			return -2;
		}
	}
	else
	{
		if(!receivedata())
		{
			if(bPrintWarnings)
				cout<<"ERROR: Cannot receive data!!"<<endl;
			return -1;
		}

	}

	if(bUseThread){
		mutex->lock();
	}

	ptr = (char*)pData;

	if(bUseThread){
		mutex->unlock();
	}

	memcpy(&MessageID, ptr, 2);
	ptr += 2;

	if (MessageID != 7)
	{
		if(bPrintWarnings)
			cout<<"\nDifferent Message ID: "<<MessageID;
		return -1;
	}

	memcpy(&nBytes, ptr, 2);
	ptr += 2;

	memcpy(&frameNumber, ptr, 4);
	ptr += 4;

	// number of data sets (markersets, rigidbodies, etc)
	nMarkerSets = 0;
	memcpy(&nMarkerSets, ptr, 4);
	ptr += 4;


	if(MarkerSetpos.size()>0)
	{
		//		cout<<"\nMarkerSet found"<<endl;
		for( long i =0; i< (long)MarkerSetpos.size(); i++)
		{
			ptr = (char*)pData;
			ptr += MarkerSetpos[i];
			//			if(bfirstrun)
			//				markerSets.push_back(markertemp);
			strcpy(markerSets[i].szMSetName, ptr);

			nDataBytes = (int) strlen(markerSets[i].szMSetName) + 1;
			ptr += nDataBytes;
			//		            printf("Model Name: %s\n", szName);

			// marker data
			memcpy(&markerSets[i].nSetMarkers, ptr, 4);
			ptr += 4;

			for(int j=0; j < markerSets[i].nSetMarkers; j++)
			{
				memcpy(&markerSets[i].MSMarkers[j].x, ptr, 4);
				ptr += 4;
				memcpy(&markerSets[i].MSMarkers[j].y, ptr, 4);
				ptr += 4;
				memcpy(&markerSets[i].MSMarkers[j].z, ptr, 4);
				ptr += 4;

				markerSets[i].MSMarkers[j].x *= 0.001;
				markerSets[i].MSMarkers[j].y *= 0.001;
				markerSets[i].MSMarkers[j].z *= 0.001;

			}
		}
	}
	else
	{
		//		cout<<"\nNo marker Set Found in data";
	}

	if(getRID.size()>0)
	{
		for(long j = 0; j<(long)getRID.size(); j++)
		{
			for(long i = 0; i<(long)RigidIDs.size(); i++)
			{
				if (getRID[j] == RigidIDs[i])
				{
					//					if (bfirstrun)
					//						rBody.push_back(rigidtemp);
					//					cout<<"\nFound Rigid Body ID: "<<getRID[j];


					ptr = (char*)pData;
					ptr += Rigidpos[i];			//going to the position of rigid body data
					memcpy(&rBody[j].ID, ptr+2, 2);
					ptr += 4;
					if(rBody[j].ID != RigidIDs[i])
					{
						cout<<"ERROR: Update Failed! RBody ID mismatch."<<endl;
						return -2;
					}
					rBody[j].set = true;

					strcpy(rBody[j].szRBodyName, Rigidname[i].c_str());
					//					cout<<"Rgdname: "<<Rigidname[i]<<endl;

					rBody[j].bMarkers = bRDetails[i];
					memcpy(&rBody[j].x, ptr, 4);
					ptr += 4;
					//Yes. Y and Z are flipped to match the reference frame of the raw marker sets.
#ifdef RBODY_FLIP_Y_Z
					memcpy(&rBody[j].z, ptr, 4);
					ptr += 4;
					memcpy(&rBody[j].y, ptr, 4);
					ptr += 4;
#else
					memcpy(&rBody[j].y, ptr, 4);
					ptr += 4;
					memcpy(&rBody[j].z, ptr, 4);
					ptr += 4;
#endif

					rBody[j].x *= 0.001;
					rBody[j].y *= 0.001;
					rBody[j].z *= 0.001;

					memcpy(&rBody[j].qx, ptr, 4);
					ptr += 4;
					memcpy(&rBody[j].qy, ptr, 4);
					ptr += 4;
					memcpy(&rBody[j].qz, ptr, 4);
					ptr += 4;
					memcpy(&rBody[j].qw, ptr, 4);
					ptr += 4;
					//Correction for converting left handed object and world frames to right handed.
					//The resulting object frame is || to world frame now.
#ifdef LHANDED_CORRECT
					rBody[j].qx *= -1.;
					rBody[j].qy *= -1.;
					rBody[j].qz *= -1.;
#endif
					// associated marker positions
					memcpy(&rBody[j].nRigidMarkers, ptr, 4);
					//					cout<<"\nNo. of Markers in RBody ID: "<<rBody[j].ID<<" = "<<rBody[j].nRigidMarkers;
					ptr += 4;
					int max = rBody[j].nRigidMarkers;
					if(rBody[j].bMarkers)		//read associated marker details
					{

						for (int i=0; i< max; i++)
						{
							memcpy(&rBody[j].RMarkers[i].x, ptr, 4);
							ptr += 4;
							memcpy(&rBody[j].RMarkers[i].y, ptr, 4);
							ptr += 4;
							memcpy(&rBody[j].RMarkers[i].z, ptr, 4);
							ptr += 4;

							rBody[j].RMarkers[i].x *= 0.001;
							rBody[j].RMarkers[i].y *= 0.001;
							rBody[j].RMarkers[i].z *= 0.001;

							//							if(bCalibrate)
							//								CalibratePos(rBody[j].RMarkers[i].x, rBody[j].RMarkers[i].y, rBody[j].RMarkers[i].z);

						}
						// associated marker IDs
						for (int i=0; i< max; i++)
						{
							memcpy(&rBody[j].RMarkers[i].nMarkerID, ptr+2, 2);
							ptr +=4;
						}
						// associated marker sizes
						for (int i=0; i< max; i++)
						{
							memcpy(&rBody[j].RMarkers[i].fMarkerSize, ptr, 4);
							ptr +=4;
						}
					}
					else
					{
						ptr += 4*5*rBody[j].nRigidMarkers;
					}
					//
					memcpy(&rBody[j].fError, ptr, 4);
					ptr += 4;
					//		                printf("Mean marker error: %3.2f\n", fError);

					break;
				}

			}
		}
	}
	else
	{
		//		cout<<"\nNo rigid bodies enabled";
	}

	if(getSID.size()>0)
	{
		for(long j = 0; j<(long)getSID.size(); j++)
		{
			for(long i = 0; i<(long)skeletonIDs.size(); i++)
			{
				if(getSID[j] == skeletonIDs[i])
				{
					//					if(bfirstrun)
					//						skeletons.push_back(skeletontemp);
					// cout<<"\nFound Skeleton ID: "<<skeletonIDs[i];
					ptr = (char*)pData;
					ptr += skeletonpos[i];
					memcpy(&skeletons[j].ID, ptr, 4); ptr += 4;
					if(skeletons[j].ID != skeletonIDs[i])
					{
						cout<<"\nSkeleton IDs mismatch. Refresh required."<<endl;
						return -2;

					}
					skeletons[j].set = true;
					strcpy(skeletons[j].szSkeletonName, skeletonname[i].c_str());
					//					skeletons[j].ID = skeletonIDs[i];

					skeletons[j].bMarkers = bSDetails[i];
					memcpy(&skeletons[j].nRigidBodies, ptr, 4); ptr += 4;

					for(int l=0; l<skeletons[j].nRigidBodies; l++ )
					{
						strcpy(skeletons[j].RBodies[l].szRBodyName, skrigidname[i][l] );
						memcpy(&skeletons[j].RBodies[l].ID, ptr, 4);
						ptr += 4;
						memcpy(&skeletons[j].RBodies[l].x, ptr, 4);
						ptr += 4;


//#ifdef RBODY_FLIP_Y_Z
//						memcpy(&skeletons[j].RBodies[l].z, ptr, 4);
//						ptr += 4;
//						memcpy(&skeletons[j].RBodies[l].y, ptr, 4);
//						ptr += 4;
//#else
						memcpy(&skeletons[j].RBodies[l].y, ptr, 4);
						ptr += 4;
						memcpy(&skeletons[j].RBodies[l].z, ptr, 4);
						ptr += 4;
//#endif
						skeletons[j].RBodies[l].x *= 0.001;
						skeletons[j].RBodies[l].y *= 0.001;
						skeletons[j].RBodies[l].z *= 0.001;

						if(bCalibrate && l==0)
						{
							lPos[0] = skeletons[j].RBodies[l].x;
							lPos[1] = skeletons[j].RBodies[l].y;
							lPos[2] = skeletons[j].RBodies[l].z;
							CalibratePos(lPos);
							skeletons[j].RBodies[l].x = lPos[0];
							skeletons[j].RBodies[l].y = lPos[1];
							skeletons[j].RBodies[l].z = lPos[2];
						}

						memcpy(&skeletons[j].RBodies[l].qx, ptr, 4);
						ptr += 4;
						memcpy(&skeletons[j].RBodies[l].qy, ptr, 4);
						ptr += 4;
						memcpy(&skeletons[j].RBodies[l].qz, ptr, 4);
						ptr += 4;
						memcpy(&skeletons[j].RBodies[l].qw, ptr, 4);
						ptr += 4;

//#ifdef LHANDED_CORRECT
//					rBody[j].qx *= -1.;
//					rBody[j].qy *= -1.;
//					rBody[j].qz *= -1.;
//#endif

						// associated marker positions
						memcpy(&skeletons[j].RBodies[l].nRigidMarkers, ptr, 4);
						//cout << skeletons[j].RBodies[l].nRigidMarkers << endl;
						ptr += 4;
						if (skeletons[j].bMarkers)		//whether to read marker details
						{
							for (int k=0; k<skeletons[j].RBodies[l].nRigidMarkers; k++)
							{
								memcpy(&skeletons[j].RBodies[l].RMarkers[k].x, ptr, 4);
								ptr += 4;
								memcpy(&skeletons[j].RBodies[l].RMarkers[k].y, ptr, 4);
								ptr += 4;
								memcpy(&skeletons[j].RBodies[l].RMarkers[k].z, ptr, 4);
								ptr += 4;

								skeletons[j].RBodies[l].RMarkers[k].x *= 0.001;
								skeletons[j].RBodies[l].RMarkers[k].y *= 0.001;
								skeletons[j].RBodies[l].RMarkers[k].z *= 0.001;
							}

							// associated marker IDs
							for (int k=0; k<skeletons[j].RBodies[l].nRigidMarkers; k++)
							{
								memcpy(&skeletons[j].RBodies[l].RMarkers[k].ID, ptr, 4);
								ptr += 4;
							}

							// associated marker sizes
							for (int k=0; k<skeletons[j].RBodies[l].nRigidMarkers; k++)
							{
								memcpy(&skeletons[j].RBodies[l].RMarkers[k].fMarkerSize, ptr, 4);
								ptr += 4;
							}
						}
						else
						{
							ptr += skeletons[j].RBodies[l].nRigidMarkers*(3+1+1)*4;
						}

						// Mean marker error
						memcpy(&skeletons[j].RBodies[l].fError, ptr, 4);
						ptr += 4;
					}
				}
			}
		}
	}
	else
	{
		//		cout<<"\nNo Skeleton enabled";
	}

	bfirstrun = false;

	ptr = (char*)pData + latencypos;
	memcpy(&latency, ptr, 4);
	ptr += 4;

	float eod = 0;
	memcpy(&eod, ptr, 4);


	//	printf("\nlatency : %10.9f\n", latency);
	//	printf("\neod     : %10.9f\n", eod);
	//	memcpy(&eod, ptr, 4); ptr += 4;
	//	printf("\nlatency2 : %10.9f\n", eod);

	//	memcpy(&eod, ptr, 4); ptr += 4;
	//	cout<<"\nEND Packet value = "<<eod<<endl;

	//	memcpy(&eod, ptr, 4); ptr += 4;
	//	cout<<"\nEND Packet value = "<<eod<<endl;
	//
	//	memcpy(&eod, ptr, 4); ptr += 4;
	//	cout<<"\nEND Packet value = "<<eod<<endl;


	return 1;
}

bool OptiTrack::SocketOpen(const char* local_ip)
{

	sd = socket(AF_INET, SOCK_DGRAM, 0);		//data socket
	sc = socket(AF_INET, SOCK_DGRAM, 0);		//command socket



	if(sd < 0)
	{
		perror("Opening datagram socket error");
		return false;
	}
	else
		printf("Opening datagram socket....OK.\n");
	/* Enable SO_REUSEADDR to allow multiple instances of this */
	/* application to receive copies of the multicast datagrams. */

	int reuse = 1;
	if(setsockopt(sd, SOL_SOCKET, SO_REUSEADDR, (char *)&reuse, sizeof(reuse)) < 0 )
	{
		perror("Setting SO_REUSEADDR error");
		close(sd);
		return false;
	}
	else
		printf("Setting SO_REUSEADDR...OK.\n");

	/* Bind to the proper port number with the IP address */

	/* specified as INADDR_ANY. */

	memset((char *) &localSock, 0, sizeof(localSock));
	localSock.sin_family = AF_INET;
	localSock.sin_port = htons(DPORT);
	localSock.sin_addr.s_addr = INADDR_ANY;
	if(bind(sd, (struct sockaddr*)&localSock, sizeof(localSock)))
	{
		perror("Binding datagram socket error");
		close(sd);
		return false;
	}
	else
		printf("Binding datagram socket...OK.\n");
	/* Join the multicast group 226.1.1.1 on the local 203.106.93.94 */
	/* interface. Note that this IP_ADD_MEMBERSHIP option must be */
	/* called for each local interface over which the multicast */
	/* datagrams are to be received. */
	//    group.imr_multiaddr.s_addr = inet_addr("239.255.42.99");
	group.imr_multiaddr.s_addr = inet_addr("239.255.42.99");
	group.imr_interface.s_addr = inet_addr(local_ip);
	if(setsockopt(sd, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&group, sizeof(group)) < 0)
	{
		perror("Adding multicast group error");
		close(sd);
		return false;
	}
	else
		printf("Adding multicast group...OK.\n");

	nlengthofsztemp = 64;		//from packetclient.cpp



	// Create a blocking, datagram socket for command socket
	if ((sc=socket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		cout<<"Command socket could not be set."<<endl;
		return false;
	}

	// bind socket
	memset(&my_addr, 0, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(0);
	my_addr.sin_addr.s_addr = INADDR_ANY;

	if (bind(sc, (struct sockaddr *)&my_addr, sizeof(struct sockaddr)) < 0)
	{
		close(sc);
		cout<<"Command socket binding error"<<endl;
		return false;
	}

	// set to broadcast mode
	ivalue = 1;
	if (setsockopt(sc, SOL_SOCKET, SO_BROADCAST, (char *)&ivalue, sizeof(ivalue)) < 0)
	{
		close(sc);
		cout<<"Command socket broadcast error"<<endl;
		return false;
	}

	isOpened = true;

	cout<<"Socket opened"<<endl;
	return true;
}
bool OptiTrack::receivedata()
{

	/* Read from the socket. */
	int datalen = sizeof(pData);
	int size;

	if(!bUseThread)
	{
		// Wait for data on socket until timeout
		struct timeval timeout;
		timeout.tv_sec = 1;
		timeout. tv_usec = 0;
		int read_result = select(sd+1, &mFdset, NULL, NULL, &timeout);
		if(read_result == -1)
		{
			cout<<"ERROR: cannot select socket!"<<endl;
			return false;
		}
		if(!read_result)
		{
			cout<<"ERROR: socket timeout occured!"<<endl;
			return false;
		}
	}

		if((size=read(sd, (char*)pData, datalen)) < 0)
		{
			cout<<"Reading datagram message error!"<<endl;
			return false;
		}

		return true;

}

bool OptiTrack::commandsocket()
{
	//	cout<<"\nCommandSocket executing..."<<endl;
	int optval = 0x100000;
	int optval_size = 4;
	setsockopt(sc, SOL_SOCKET, SO_RCVBUF, (char *)&optval, 4);
	//    getsockopt(sc, SOL_SOCKET, SO_RCVBUF, (char *)&optval, &optval_size);

	getsockopt(sc, SOL_SOCKET, SO_RCVBUF, (char *)&optval, (socklen_t *)&optval_size);


	memset(&HostAddr, 0, sizeof(HostAddr));
	HostAddr.sin_family = AF_INET;
	HostAddr.sin_port = htons(CPORT);
	HostAddr.sin_addr.s_addr = INADDR_BROADCAST;		//broadcast to all hosts

	// send initial ping command
	sPacket PacketOut;
	//	PacketOut.iMessage = NAT_PING;
	//	PacketOut.nDataBytes = 0;
	//	nTries = 3;
	//	while (nTries--)
	//	{
	//		int iRet = sendto(sc, (char *)&PacketOut, 4 + PacketOut.nDataBytes, 0, (sockaddr *)&HostAddr, sizeof(HostAddr));
	//		if(iRet <0)
	//			break;
	//	}

	PacketOut.iMessage = NAT_REQUEST_MODELDEF;
	PacketOut.nDataBytes = 0;
	nTries = 3;
	while (nTries--)
	{
		int iRet = sendto(sc, (char *)&PacketOut, 4 + PacketOut.nDataBytes, 0, (sockaddr *)&HostAddr, sizeof(HostAddr));
		if(iRet <0)
			break;
	}
	int addr_len;
	int nDataBytesReceived;
	//	    char str[256];
	sockaddr_in TheirAddress;
	sPacket PacketIn;
	addr_len = sizeof(struct sockaddr);

	nTries = 3;
	while(nTries--)
	{

		nDataBytesReceived = recvfrom( sc,(char *)&PacketIn, sizeof(sPacket), 0, (struct sockaddr *)&TheirAddress, (socklen_t *)&addr_len);

		if((nDataBytesReceived == 0) || (nDataBytesReceived == SOCKET_ERROR) )
			continue;

		if(PacketIn.iMessage == NAT_PINGRESPONSE)
		{
			cout<<"\n Natnet Version: "<<(int)PacketIn.Data.Sender.NatNetVersion[0]<<"."
					<<(int)PacketIn.Data.Sender.NatNetVersion[1]<<"."
					<<(int)PacketIn.Data.Sender.NatNetVersion[2]<<"."
					<<(int)PacketIn.Data.Sender.NatNetVersion[3]<<endl;
			cout<<"\n Server Version: "<<(int)PacketIn.Data.Sender.Version[0]<<"."
					<<(int)PacketIn.Data.Sender.Version[1]<<"."
					<<(int)PacketIn.Data.Sender.Version[2]<<"."
					<<(int)PacketIn.Data.Sender.Version[3]<<endl;
			return false;
		}

		else if(PacketIn.iMessage!=NAT_MODELDEF)
		{
			cout<<"\n Model Definition not received. Retrying..."<<endl;
			cout<<"\n packetin message:"<<PacketIn.iMessage;

			continue;
		}
		//		cout<<"\nPacket message: "<<PacketIn.iMessage<<endl;
		ptr = (char*)&PacketIn;
		//		cout<<"Command socket executed successfully."<<endl;
		break;


	}

	return true;

}
