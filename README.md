# vision_pkg

# Motion Capture at LASA using Optitrack.

This package streams ROS tf frames using the optitrack data.

# Instructions

## Installation

Clone the repository into your catkin workspace and run `catkin_make`.

## Optitrack computer setup

 1. Open program
 2. Load a project

## Basic functionality test

To check basic functionality
will determine

rosrun vision_pkg TesterOP


### Basic test program

To test basic functionality and connection with the Windows server the following program may be used

    roscd VisionPkg
    ./bin/Tester

This should output a list of IP addresses that your machine has and ask you to choose one of those. Select the one that looks like “128.178.....”. If not, check if the streaming is switched on from the Arena software.

Next, it should output a list of object names that are being streamed by the server. Select the object you want to read and press Enter. It should start printing the current position and orientation of that object.

### Streaming over ros-tf

ROS provides a ready to use environment for streaming and reading transformations over a topic. This is provided by the ros package “tf”. The VisionPkg provides a node to stream all available (or chosen) objects over the topic "/tf". To do this, simply run the following

    roslaunch VisionPkg example.launch

There are some options that should be specified correctly in the file “example.launch”. The default ones (specially the ip) might not work for your case. The options to be set are as follows:

``` xml
<param name="local_ip" value="128.178.145.139"/>
```

The local\_ip must be specified correctly as above. This is the IP of your linux computer (not the Windows server!).

``` xml
<param name="use_thread" value="false"/>
```

Instruct the node to read the socket in a separate thread. Avoids blocking if there is no new data coming. Setting this to “true” will stream the last known value continuously over ROS until a new one is available. If set to “false”, it waits until a new value is available and nothing is sent to the ROS topic until then.

``` xml
<param name="publish_frequency" value="250"/>
```

Specify the frequency of publishing frames as above.

``` xml
<param name="obj_list" value="myobj1_root myobj2_root"/>
```

Specify which objects you wish to track as a whitespace-separated list. If this tag is commented or removed, all available objects are tracked.

``` xml
<param name="calib_file" value="mycalibfile.txt"/>
```

Specify the path of the calibration file that contains the transformation between the vision system origin and the robot base. This is helpful


If no calibration file is provided, there is not robot frame streamed. Values are available only with respect to the vision system's origin.

### Reading from ros-tf

Once the frames are streaming over the ros-tf network, they can be read through code as follows:

#### Before ROS-Fuerte

If your version of ROS is older than “Furerte” you should do the following. Your project must include the “tf” ros package as a dependency in your “manifest.xml” file. To do this add the line

``` text
<depend package="tf"/>
```

to your project's manifest.xml. Include the required header file as:

``` cpp
#include <tf/transform_listener.h>
```

In your source code, the following objects are required in order to read the transforms from tf. They should go either in the header file or be global variables.

``` cpp
// Transform listener from ROS
tf::TransformListener *listener;

// Transform object from ROS
tf::StampedTransform transform;

// 3x3 Matrix object from bullet. To hold the rotation matrix from vision.
btMatrix3x3 btmat;

// Vector3 object from bullet. To hold the origin vector from vision.
btVector3 btVec3;

// If you are using MathLib. 4x4 Matrix object to hold the homogeneous transformation
MathLib::Matrix4 visionHMatrix;
```

The following code reads the most recent transformation from the robot's frame to the object's frame.

``` cpp
// Reading the transform from the tf topic
listener->lookupTransform("/robot", "/myobjname_root", ros::Time(0), transform);

// Separating the rotation matrix and origin vector
btmat.setRotation(transform.getRotation());
btvec3 = transform.getOrigin();

// Filling up a 4x4 Homogeneous transformation matrix from the bullet datatypes above
visionHMatrix.Set(btmat[0][0], btmat[0][1], btmat[0][2], btvec3[0],
          btmat[1][0], btmat[1][1], btmat[1][2], btvec3[1],
          btmat[2][0], btmat[2][1], btmat[2][2], btvec3[2],
          0,               0,              0,           1     );
```

#### After ROS-Fuerte

If your ROS version is newer or equal to “Fuerte”, the objects of type btVector3 and btMatrix3x3 must be changed to

``` cpp
// 3x3 Matrix object from tf. To hold the rotation matrix from vision.
tf::Matrix3x3 btmat;

// Vector3 object from tf. To hold the origin vector from vision.
tf::Vector3 btVec3;
```

#### After ROS-Hydro

If your ROS version is newer or equal to “Hydro”, the specification of dependencies should be made according to the catkin build system. More information on it can be found on the [ROS-catkin] wiki.

#### Reading from commandline

If you simply want to check the locations of some frames visually, you may run the following command

``rosrun tf tf_echo /robot /myobjname_root``

to print out the position and orientation of the frame "myobjname_root" with respect to the frame "robot".

Library for reading motion capture data.
