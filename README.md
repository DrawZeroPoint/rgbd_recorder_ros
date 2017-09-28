## A synchronized rgbd video recorder based on ROS
## Usage
1. Plug in your rgbd camera and open it with command:

 For Xtion: `roslaunch openni2_launch openni2.launch`

 For Astra: `roslaunch astra_launch astra.launch`

 You may modify launch files above to make sure they set params like this (Default value for these args are false):
 ```
  <!-- Hardware depth registration -->
  <arg name="depth_registration" default="true" />

  <!-- Driver parameters -->
  <arg name="color_depth_synchronization" default="true" />

 ```
 Notice that these values can also be changed using dynamic reconfigure (rqt)

2. Run the recorder with command:

 `roslaunch rgbd_recorder_ros recorder.launch`

 Before you run this, make sure the 'path' is set correctly, in where the captured videos are stored.

3. Use ros_param "/video_num" to get new videos:

 `rosparam set /video_num VALUE`

  where VALUE need to be given by user, which starts with 1. After one video was recorded, it's recommanded increasing the VALUE by 1

