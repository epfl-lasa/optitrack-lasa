#include "OptiTrack.h"
#include "Calibrator.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <unistd.h>
#include <netdb.h>
#include <fstream>

#include <ifaddrs.h>
#include <net/if.h>
#include <sys/ioctl.h>
using namespace std;

int main(int argc, char** argv)
{
  struct ifaddrs * ifAddrStruct=NULL;
  struct ifaddrs * ifa=NULL;
  void * tmpAddrPtr=NULL;

  getifaddrs(&ifAddrStruct);
  vector<string> iplist;
  int count=0;
  string localip="";
  char buf[1025];
  int choice;

//user can specify an ip on the command line or try to automaticcaly find one if no command line argument is given
  if(argc > 1) {
    localip = argv[1];
  } else {


    cout<<"Detecting IP Addresses..."<<endl;
    cout<<"==========================================================="<<endl;
    for (ifa = ifAddrStruct; ifa != NULL; ifa = ifa->ifa_next) {
      if (ifa ->ifa_addr->sa_family==AF_INET) { // check it is IP4
        // is a valid IP4 Address
        tmpAddrPtr=&((struct sockaddr_in *)ifa->ifa_addr)->sin_addr;
        char addressBuffer[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, tmpAddrPtr, addressBuffer, INET_ADDRSTRLEN);
        printf("%d)  %s IP Address %s\n", ++count, ifa->ifa_name, addressBuffer);
        iplist.push_back(addressBuffer);
      }
    }
    printf("%d)  Type Manually\n", ++count);
    cout<<"==========================================================="<<endl;

    sprintf(buf, "Choose your ip from the above list [1-%d]: ", count);
    choice = -1;
    while(choice <1 || choice > count) {
      cout<<buf;
      cin>>choice;
    }

    if(choice == count) {
      cout<<"Your local ip is : ";
      cin>>localip;
    } else
      localip = iplist[choice-1];
  }
  cout<<"Using IP "<<localip<<endl;

  OptiTrack track1;


  //local pc ip

  //	if(track1.Init("128.178.145.82")<0)

  if(track1.Init(localip.c_str())<0)
    cout<<"ERROR: Cannot initialize"<<endl;

//	track1.loadCalibrationMatrix("KUKA_VisionCalib.txt");
//	track1.DisplayInfo();
  cout<<"Detecting Rigid Bodies..."<<endl;
  vector<string> namelist = track1.GetRBodyNameList();

  if(namelist.size() == 0) {
    cout<<"No rigid bodies found!!"<<endl;
    return 1;
  }

  cout<<"==========================================================="<<endl;
  for(unsigned int i=0; i<namelist.size(); i++)
    cout<<i+1<<")  "<<namelist[i]<<endl;
  cout<<"==========================================================="<<endl;

  sprintf(buf, "Choose an object to track [1-%d]: ", (int)(namelist.size()));
  choice = -1;
  while(choice <1 || choice > (int)namelist.size()) {
    cout<<buf;
    cin>>choice;
  }
  string totrack = namelist[choice-1];
  string calibfile = "KUKAVisionCalib.txt";
  track1.enableRBody(totrack.c_str(), false);

  track1.loadCalibrationMatrix(calibfile.c_str());
  double pos[3];
  double orient[3][3];

  cout<<"Press enter to start..."<<endl;
  getchar();
  getchar();
  while(true) {
    track1.Update();
    track1.getRBodyPosition(pos, totrack.c_str());
    track1.getRBodyOrientation(orient, totrack.c_str());
    printf("Pos:\n%3.5lf %3.5lf %3.5lf\n", pos[0], pos[1], pos[2]);

    printf("Orient:\n%3.5lf %3.5lf %3.5lf\n%3.5lf %3.5lf %3.5lf\n%3.5lf %3.5lf %3.5lf\n",
           orient[0][0], orient[0][1], orient[0][2],orient[1][0], orient[1][1],
           orient[1][2],orient[2][0], orient[2][1], orient[2][2] );

    usleep(1000);

  }

  return 0;


}

