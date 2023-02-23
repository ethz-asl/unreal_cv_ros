# unreal_cv_ros 

**unreal_cv_ros** is a package to allow ROS based simulation of a MAV equipped with a 3D-reconstruction sensor. The simulation is performed inside an [Unreal Engine 4](https://www.unrealengine.com/en-US/what-is-unreal-engine-4) (UE4) game. The node-game communcation is carried out utilizing the [UnrealCV](https://github.com/unrealcv/unrealcv) computer vision plugin for UE4.

* **Note:** We recommend also checking out our recent robot simulator [https://github.com/ethz-asl/unreal_airsim](https://github.com/ethz-asl/unreal_airsim) for UE4 based on Microsoft [AirSim](https://microsoft.github.io/AirSim/) with more features and higher performance.

# Table of Contents
**Installation**
* [Dependencies](#Dependencies)
* [Installation](#Installation)
* [Data Repository](#Data-Repository)

**Examples**
* [Run in test mode](#Run-in-test-mode)
* [Run with MAV](#Run-with-MAV)

**Documentation: ROS Nodes**
* [unreal_ros_client](#unreal_ros_client)
* [sensor_model](#sensor_model)
* [simulation_manager](#simulation_manager)

**Documentation: Working with Unreal**
* [UnrealCV Plugin Setup](#UnrealCV-Plugin-Setup)
* [Creating UE4 worlds](#Creating-UE4-worlds)
* [When to use which mode](#When-to-use-which-mode)
* [Pawns and Cameras](#Pawns-and-Cameras)
* [Custom collision and camera settings](#Custom-collision-and-camera-settings)
* [Static mesh collision](#Static-mesh-collision)
* [Producing ground truth pointclouds](#Producing-ground-truth-pointclouds)
* [The Unreal Coordinate System](#The-Unreal-Coordinate-System)

**Troubleshooting**
* [Frequent Issues](#Troubleshooting)

# Installation
## Dependencies
**System Dependencies:**

Unreal_cv_ros requires the UnrealCV python library: `pip install unrealcv`.

**ROS Packages:**

The perception functionalities (unreal_ros_client + sensor_model) depend on:
* `catkin_simple` ([https://github.com/catkin/catkin_simple](https://github.com/catkin/catkin_simple))

To run the full MAV simulation, these additional packages are needed: 
* `rotors_simulator` ([https://github.com/ethz-asl/rotors_simulator](https://github.com/ethz-asl/rotors_simulator))
* `mav_control_rw` ([https://github.com/ethz-asl/mav_control_rw](https://github.com/ethz-asl/mav_control_rw))
* `voxblox` ([https://github.com/ethz-asl/voxblox](https://github.com/ethz-asl/voxblox))

## Installation
**Install the ROS package:**

Installation instructions on Linux:

1. Move to your catkin workspace: 
```shell script
cd ~/catkin_ws/src
```
2. Install using a SSH key or via HTTPS: 
```shell script
git clone git@github.com:ethz-asl/unreal_cv_ros.git # SSH
git clone https://github.com/ethz-asl/unreal_cv_ros.git # HTTPS
```
3. Compile: 
```shell script
catkin build unreal_cv_ros
```
**Install Unreal Engine:**

To create custom worlds or run non-shipping projects, the Unreal Engine Editor is used. On Linux, this requires installing Unreal Engine and building it from source. Complete installation instrcutions are given on [their webpage](https://docs.unrealengine.com/en-US/SharingAndReleasing/Linux/BeginnerLinuxDeveloper/SettingUpAnUnrealWorkflow/index.html). In addition, install the UnrealCV plugin to Unreal Engine as described in [UnrealCV Plugin Setup](#UnrealCV-Plugin-Setup).

The unreal\_cv\_ros pipeline also works with stand-alone games that were built including the UnrealCV plugin (e.g. the examples below), which do not require additional installations.


## Data Repository
Other related resources, such as blueprint classes and experiment scenarios, can be downloaded from [here](https://drive.google.com/drive/folders/1wIjnqXlGjrCOQdkQn31Ksep88d18O2u7?usp=sharing). This repo and the related resources were developped and tested with unrealcv v0.3.10 and Unreal Engine 4.16.3.

# Examples
## Run in test mode
To illustrate the vision pipeline in stand-alone fashion, we run the unreal_ros_client in test mode with ground_truth as our sensor model. 

![uecvros_ex_test](https://user-images.githubusercontent.com/36043993/52844470-9028de00-30fc-11e9-975f-0b204ae52f6d.png)
View of the realistic rendering demo (left) and the produced ground truth point cloud with corresponding camera pose (right).

Please download the [RealisticRendering](http://docs.unrealcv.org/en/master/reference/model_zoo.html#rr) game binary and launch the game. In a command window type `roslaunch unreal_cv_ros example_test.launch` to start the pipeline and wait until the connection is setup (takes few seconds). You can now navigate the camera inside the game using the mouse and W-A-S-D keys while a rviz window displayes the produced ground truth pointclouds as well as the camera pose in the unreal world frame.

Note that the test mode is mostly meant to explore the effects of different sensor models on the pointcloud. Since the taking of images and read-out of the unreal-pose is done sequentially, fast movement in the game may result in some frames not being well aligned.

## Run with MAV
This example demonstrates how to setup the full scale MAV simulation. It uses unreal_cv_ros for perception and collision checking, gazebo to model the MAV physics, a MPC high level and PID low level controller for trajectory tracking and voxblox to integrate the pointclouds into a map. 

![uecvros_ex_mav](https://user-images.githubusercontent.com/36043993/52851608-814b2700-310e-11e9-9b41-256d11898bbc.png)
Voxblox map that is continually built as the MAV follows a trajectory through the room.

Run the RealisticRendering demo as in the previous example. Make sure to tab out of it immediately to disable player control. This is necessary because unreal_cv_ros sets its world frame at the current MAV position on startup and this example expects the game demo to be in its initial state. Furthermore, if captured, unreal engine continues to accept player input that may interfere with the simulation. 

In a command window type `roslaunch unreal_cv_ros example_mav.launch` to start the pipeline. The simulation\_manager will supervise the startup of all elements (this may take few seconds). After everything set up cleanly, the example node will publish two trajectory segments, which the MAV tries to follow to explore the room. In a rviz window, the current MAV pose is depicted together the planned trajectories and the voxblox mesh representation of the room as it is explored.

# Documentation: ROS Nodes
## unreal_ros_client
This node manages the unrealcv client and the connection with a running UE4 game. It sets the camera position and orientation within the unreal game and produces the raw image data and camera calibration used by the sensor_model.

### Parameters
* **mode** In which mode the client is operated. Currently implemented are:
  * **test** Navigate the MAV manually in the unreal game. The client will periodically publish the sensor data.
  * **standard** The camera pose is set based on the `/odometry` topic and images are taken. Uses the default unrealcv plugin, operates at ~1 up to 2 Hz.
  * **fast** Similar to standard, but requires the custom unrealcv plugin (See [Unrealcv Plugin Setup](#Unrealcv-Plugin-Setup)).  Operates at ~3 up to 5 Hz.
  
  Default is 'standard'.
* **collision_on** Set to true to check for collision in the unreal game. Set to false to set the camera anyway. May result in rendering artifacts if the camera overlaps with objects. Default is true.
* **collision_tol** This parameter only shows in `standard` mode. Collision warnings are triggered if the requested and realized position are further apart than this threshold (in unreal units, default unit is cm). Default is 10.
* **publish_tf** If true, the client pulishes a tf-transform of the camera pose for every taken image with a matching timestamp. Default is False.
* **slowdown** Artificially slows down the time between setting the pose and taking images to give unreal engine more time to render the new view. Slowdown is expected as wait duration in seconds wall-time. Default is 0.0.
* **camera_id** Lets unrealcv know which camera to use. Default is 0.
* **queue_size** Queue size of the input subscriber. Default is 1.
* **Camera Parameters:** To change the resolution and field of view (FOV) of the camera, the [unrealcv configuration file](http://docs.unrealcv.org/en/master/plugin/config.html) needs to be changed. The relevant path is displayed when the unreal_ros_client node is launched. When the client is setup, these values are published as 'camera_params' on the ros parameter server for other nodes to access them.

### Input Topics
* **odometry** of type `nav_msgs.msg/Odometry`. Set the camera pose w.r.t. its position and yaw at the connection of the client. Does not appear in `test` mode.

### Output Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. The output of the in-game image capture, containing a color and a depth image encoded as npy binaries.
* **collision** of type `std_msgs.msg/String`. Publishes "MAV collision detected!" upon collision detection. Only available if collision_on is true.

### Services
* **terminate_with_reset** of type `std_srvs.srv/SetBool`. Stop processing new odom requests and reset the camera to the initial pose.

## sensor_model
This node converts the unreal_ros_client output into a pointcloud for further processing. Sensor specific behaviour can be simulated by artificially altering the ground truth data.

### Parameters
* **model_type** Which sensor to simulate. Currently implemented are: 
  * **ground_truth:** Produces the ground truth pointcloud without additional processing.
  * **kinect:** Simulates a Kinect 3D sensor according to [this paper](https://ieeexplore.ieee.org/abstract/document/6375037). Notice that the sensor range is cropped to \[0.5, 3.0\] m and that the inclination angle is neglected, since it is constant up until ~60, 70 degrees.
  * **gaussian_depth_noise:** Apply a gaussian, depth dependent term to the pointcloud z coordinate. Allows to set the params `k_mu_<i>` and `k_sigma_<i>` for i in \[0, 3\], where f(z) = k0 + k1 * z + k2 * z^2 + k3 * z^3. Default for all k is 0.0.
  
  Default is 'ground_truth'.
* **camera_params_ns** Namespace where to read the unreal camera parameters from, which are expected as {height, width, focal_length}. Notice that the sensor_model waits until the camera params are set on the ros parameter server (e.g. from the unreal\_ros\_client). Default is 'unreal\_ros\_client/camera_params'.
* **maximum_distance** All points whose original ray length is beyond maximum_distance are removed from the pointcloud. Set to 0 to keep all points. Default is 0.0.
* **flatten_distance** Sets the ray length of every point whose ray length is larger than flatten\_distance to flatten\_distance. Set to 0 to keep all points unchanged. Default is 0.0.
* **publish_color_images** In addition to the point clouds publish the perceived images, encoded as 4 channel RGBA. Default is False.
* **publish_gray_images** If true publish a gray scale image with every pointcloud (e.g. for integration with rovio). Default is False.

### Input Topics
* **ue_sensor_raw** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client that is to be further processed.

### Output Topics
* **ue_sensor_out** of type `sensor_msgs.msg/PointCloud2`. Resulting pointcloud after applying the simulated sensing pipeline.
* **ue_color_image_out** of type `sensor_msgs.msg/Image`. Color image of the current view. Only published if publish_color_images is true.
* **ue_gray_image_out** of type `sensor_msgs.msg/Image`. Gray scale image of the current view. Only published if publish_gray_images is true.


## simulation_manager
This node is used to launch the full MAV simulation using gazebo as a physics engine and an unreal game for perception and collision modeling. It is used to coordinate simulation setup and monitor the unreal_cv_ros pipeline performance.

### Parameters
* **ns_gazebo** Namespace of gazebo, including the node name. Default is '/gazebo'.
* **ns_mav** Namespace of the MAV, which is expected to end with the actual MAV name. Default is '/firefly'.
* **monitor** Set to true to measure the unreal vision pipeline's performance. Default is False.
* **horizon** How many datapoints are kept and considered for the performance measurement. Only available if monitor is true. Default is 10.

### Input Topics
* **ue_raw_in** of type `unreal_cv_ros.msg/UeSensorRaw`. Output of the unreal\_ros\_client for performance measurements. Only available if monitor is true.
* **ue_out_in** of type `sensor_msgs.msg/PointCloud2`. Output of the sensor model for performance measurements. This topic needs to be matched to check for correct startup.

### Output Topics
* **simulation_ready** of type `std_msgs.msg/String`. After successful start of the pipeline publishes "Simulation Ready" to let other nodes start or take over.

### Services
* **display_monitor** of type `std_srvs.srv/Empty`. Print the current monitoring measurements to console. Only available if monitor is true.


# Documentation: Working with Unreal

## UnrealCV Plugin Setup
### Standard Plugin
For the modes `test` and `standard` the default UnrealCV plugin is sufficient. Install it to the project or engine as suggested on [their website](http://docs.unrealcv.org/en/master/plugin/install.html). 

### Adding the 'vget /uecvros/full' command
This additional command is required to operate in`fast` mode.
* **Compiled Plugin** The compiled plugin can be downloaded from the [data repository](#Data-Repository).

* **Compile it yourself**  To compile the plugin, first create an unreal engine development environment as is explained [here](http://docs.unrealcv.org/en/master/plugin/develop.html). Then change the source code of the unrealcv plugin in your project (eg. `UnrealProjects/playground/Plugins/UnrealCV/Source/UnrealCV/Private/Commands`) to include the new command:
  - In `CameraHandler.h` add the command declaration
  - In `CameraHandler.cpp` add the command dispatcher and function body. 
  
These 3 code snippets can be copied from `unreal_cv_ros/content/CustomPluginCode.cpp`. Afterwards build the project as explained in the unrealcv docs. The compiled plugin can now be copied to other projects or the engine for use.

### Command reference 
This command take images and then request the passed position and orientation. This leaves time for the engine to finish rendering before the next images are requested.
* Syntax: `vget /uecvros/full X, Y, Z, p, y, r, collision, cameraID`. All argumentes, except the cameraID which is a uint, are floats. All arguments must be set. 
* Input: 
  - (X, Y, Z): New target position in unreal units.
  - (p, y, r): New target orientation in degrees (and unreal coordinate system).
  - collision: Set to 1 to check collision, -1 to ignore it.
  - cameraID:  ID of the camera to use in compliance with the unrealcv structure. Generally use 0.
* Output: 
  - In case of collision returns "Collision detected!" as unicode. 
  - Otherwise returns the stacked binary image data as string, where the first half representes the color image and the second half the depth image, both as npy arrays.

## Creating UE4 worlds
In order to easily create unreal_cv_ros compatible  UE4 worlds:
* Install and use unreal engine editor **4.16** for compatibility with unrealcv. (Note: Newer versions *should* work too but without guarantees).
* Make sure the unrealcv plugin is installed **and** activated in the current project (In the Editor check: Edit > Plugins > Science > Unreal CV, see [unrealcv docs](http://docs.unrealcv.org/en/master/plugin/install.html)).
* Set the player pawn to a flying spectator type with collision: World Settings > Game Mode > Selected GameMode > Default Pawn Class := UecvrosDefaultPawn. (If this is read-only just change to a custom GameMode.) 
  - Highly recommended: If using the `fast` mode, use the `UecvrosDefaultPawn`. The blueprint class can be downloaded from the [data repository](#Data-Repository).
  - Otherwise use the unreal engine built in `DefaultPawn`.
  
## When to use which mode
The unreal_cv_ros plugin is designed to work with all sorts of unreal worlds, however performance depends on how the world and the plugin is setup:

| Plugin | Unreal World | Method to use |
| --- | --- | --- |
| Custom | Custom | When the plugin and unreal world are modifiable, it is highly recommended to use the `fast` mode with the `UecvrosDefaultPawn`. This is not only the fastest combination, but it is also more stable and produces accurate and well-aligned pointclouds. |
| Custom | Static | If the unrealworld cannot be modified (usually it uses the `DefaultPawn`), it is still recommended to use the `fast` mode, since it runs considerably faster. However, the default camera may pose obstacles, such as motion blurr, that hinders alignment of the recorded images with the requested pose. Adjust the 'slowdown' parameter of the unreal_ros_client to give unreal enough time to properly process the desired images. |
| Static | Any | If the plugin cannot be modified, use the `standard` mode and the built in `DefaultPawn`. |
| `test` mode | Any | If you want to run the `test` mode, use the built in `DefaultPawn` to allow for suitable user control. |

## Pawns and Cameras 
Unrealcv works by capturing pawns and cameras in the unreal game, to which it attaches. Pawns are the physical entities that move around and collide with things, whereas cameras typically provide the views. Usually, pawns are not placed in the world, thus upon game start a Pawn will spawn at PlayerStart that is then possessed by player control and unrealcv.
* Cameras in unrealcv are stored in chronological order. If you want to access different cameras use the 'camera_id' param of the unreal_ros_client.
* The `DefaultPawn` is incoporates standard flying player control and a default view. However that incorporates all default features such as motion blurr when moving (or teleporting!) the pawn.
* The `UecvrosDefaultPawn` inherits from the `DefaultPawn`. Furthermore, it has a camera attached to it that can be modified. However this disables the default player camera orientation control (which is also the one accessed with the standard unrealcv plugin).

Custom camera settings can similarly be changed by editing the camera component. Notice that unrealcv overwrites certain camera settings (such as the resolution and field of view) using the unrealcv configuration file.

## Custom collision and camera settings
The default collision for the `DefaultPawn` and `UecvrosDefaultPawn` is a sphere of 35cm radius. For custom collision and camera, you need to create your own pawn blueprint (which inherits from the respective base class). E.g. an 'easy' way to create a custom `DefaultPawn` is as follows:
1. In the Modes window, search for 'DefaultPawn' and create an instance (drag and drop into the game world).
2. Select the pawn instance and click Blueprints > Convert selected actor to blueprint class...
3. Save the new class, e.g. in the content/blueprints folder as myDefaultPawn.
4. The class should now open in the blueprint editor, where its components can be edited.
5. To change the radius select the 'CollisionComponent', and under Details > Shape > SphereRadius := myValue. Notice that too short radii can allow the camera to enter certain objects, creating graphical artifacts.
6. Save the blueprint. Set the World Settings > Game Mode > Selected GameMode > Default Pawn Class := myDefaultPawn

## Static mesh collision
When creating UE4 worlds, it is worth double checking the player collision with static mesh actors (the scene). By default, unreal produces convex hulls as simple collision objects and checks against simple collision. However, this may result in faulty collision detection. Collision can be changed to match the visible mesh as follows (may degrade performance): 
1. Right click your object in the World Outliner and select "Edit 'your object'".
2. Set Collision > Collision Complexity := Use Complex Collision As Simple.
3. Save and close the editing window.

Current collision can be visualized by changing the viewmode in the unreal editor from "Lit" to "Player Collision". 

## Producing ground truth pointclouds
For simulation evaluation, ground truth meshes can be exported from the unreal editor and further processed using tools such as [CloudCompare](https://www.danielgm.net/cc/). A voxblox compatible ground truth pointcloud can be generated as follows:
* In the **Unreal Engine Editor**,
1. Select the objects of interes in the World Outliner
2. File > Export Selected...
3. Save as \*.obj file. (Currently no need to export the material too.)
  
* Open **CloudCompare**,
4. File > Import > my_mesh_export.obj
5. Use 'Edit > Multiply/Scale > 0.01' to compensate for unreal engine units (default is cm).
6. Use 'Edit > Apply transformation' to place and rotate the object relative to the PlayerStart settings from unreal. (I.e. with the origin at the PlayerStart location, x pointing towards the PlayerStart orientation, y-left and z-up).
7. Use 'Edit > Crop' to remove meshes outside the region of interest and unreachable points, such as the ground plane.
8. Click 'Sample points on a mesh' to create a pointcloud. (Currently no need to generate normals or color.)
9. 'File > Save' and save as a \*.ply in ASCII format
  
* With a **TextEditor**,
10. Open my_gt_pointcloud.ply and remove the "comment Author" and "obj_info" fields (lines 3 and 4, these create errors with pcl::plyreader).

## The Unreal Coordinate System
For application with the unreal\_ros\_client, the coordinate transformations are already implemented so no need to worry. For development/debugging tasks: Unreal and unrealcv use the following coordinate system: 

* **Unreal World** The default coordinate system is X-forward, Y-right, Z-up. Default units are cm.
* **Rotation Direction** Positive rotation directions around the unreal world coordinate axes are mathematically positive around the X and Y axes and negative around Z axis.
* **Rotation parametrization** The Unreal engine interface and therefore also the unrealcv commands parse rotations as pitch-yaw-roll (pay attention to the order). However, inside the engine rotations are performed as Euler-ZYX rotations (i.e. roll-pitch-yaw). Default units are degrees.

# Troubleshooting
Known Issues and what to do about them:
1. **Error message "Error addressing the unrealcv client. Try restarting the game.":**
    - Make sure that only a single unreal game **or** editor is running. When using the editor, the unrealcv plugin is loaded already during startup (without the game itself running!). Since unrealcv per default connects to `localhost:9000` the server will be blocked if any other instance of it is running. 
2. **The produced pointclouds are not well aligned with the requested pose / are smeared out:**
    - If running in standard mode, consider switching to fast mode which is not only faster but also more stable. (In standard mode every orientation change is carried out as rotation movement inducing rotation offsets (such as motion blurr when using the DefaultPawn camera), especially for high rotation rates).
    - Give unreal engine more time for rendering and settling by adjusting the `slowdown` paramter of the unreal\_ros\_client. (This problem typically occurs for low rendering/frame rates in the unreal game or when using the DefaultPawn).
3. **I get strange collision warnings where there shouldn't be any:**
    - Make sure you are not using simple collision, which can greatly inflate collision bounds of objects. See [here](#Static-mesh-collision) for more info.
4. **I get collision warnings for quickly executed trajectories:**
    - The unreal teleport action with collision checking sweeps (as far as I know) a linear path between every request. Try increasing the unreal\_ros\_client's update rate or slowing down simulated time.
5. **Installing UE4.16: "xlocale.h not found":**
    - xlocale is no longer needed and this bug is fixed in future versions of UE4, removing the include statements of xlocale should fix the issue.
