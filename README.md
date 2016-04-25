# Motion Capture at LASA using Optitrack.

This package streams ROS tf frames using the optitrack data.

# Instructions

## Installation

Clone the repository into your catkin workspace and do `catkin_make`.

## Starting the Windows optitrack arena

 1. Open the Arena program
 2. Load your project: this contains camera calibration and markers
 3. In the bottom, click the 'capture' tab and click play. You should see your
    markers detected in the GUI
 4. On the right (under Settings) click the Stream tab. The IP address should be
    the IP address *of the windows computer* (that is running Arena). De-select
    "3D marker positions" and click "Start Streaming Frames".

The Optitrack (windows) computer streams the frames over multicast.
You do not need to specify the IP address of the receiving computer.

## Running a basic functionality check on your computer

This test will determine whether or not you are receiving the optitrack data.

 1. Run the TesterOP program: `rosrun vision_pkg TesterOP`
 2. Put in *your own computer's IP address* when asked (you can probably just
    select option 2 to automatically select it).
 3. Select a marker to track from the list of available optitrack markers.
 4. Press enter, you should see streaming data.

If you do not open a connection or see any markers, make sure that:

 - You put in your own IP address (the one for the computer running the program)
   in the IP field
 - The Windows computer is streaming data
 - The optitrack arena is seeing one or more markers.

## Streaming tf transforms using a ROS node

Launch the optitrack node with the IP address of *your computer running the ROS
node*. Note this is **not** the IP of the windows computer.

    roslaunch vision_pkg example.launch local_ip:=128.178.145.XXX

This will stream transforms for all available markers.

## Tips and tricks

You can view the frames being published using:

    rosrun tf tf_monitor

Alternatively, run view_frames will generate a PDF of all the TF frames.
`rviz` is another good option to see the frames.

See the [tf tutorial](http://wiki.ros.org/tf/Tutorials) for lots more information.

The Arena software on the windows computer has a tendency to crash.

### Other launch file options

NOTE: This section needs to be updated.

Instruct the node to read the socket in a separate thread. Avoids blocking if
there is no new data coming. Setting this to “true” will stream the last known
value continuously over ROS until a new one is available. If set to “false”, it
waits until a new value is available and nothing is sent to the ROS topic until
then.

``` xml
<param name="publish_frequency" value="250"/>
```

Specify the frequency of publishing frames as above.

``` xml
<param name="obj_list" value="myobj1_root myobj2_root"/>
```

Specify which objects you wish to track as a whitespace-separated list. If this
tag is commented or removed, all available objects are tracked.

``` xml
<param name="calib_file" value="mycalibfile.txt"/>
```

Specify the path of the calibration file that contains the transformation
between the vision system origin and the robot base. This is helpful


If no calibration file is provided, there is not robot frame streamed. Values
are available only with respect to the vision system's origin.
