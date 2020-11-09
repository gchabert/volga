

Acknowledgement 
===============
This development is supported by [ROSIN](rosin-project.eu]) - ROS-Industrial Quality-Assured Robot Software Components.

The ROSIN project has received funding from the European Union’s Horizon 2020
research and innovation programme under grant agreement No. 732287. 

Install & Build 
===============

The Volga packages have been developed and tested under ROS Melodic.
Portabilty to other versions of ROS is not guaranteed so far.

The Volga packages require two external libraries that must be installed by hand: Ibex and OpenMesh.

This section is only for installing and building the core package. See below for usage with Moveit.

Ibex
-------
- Download Ibex from [www.ibex-lib.org](www.ibex-lib.org).

    The current version is compatible with **Release 2.8** and subsequent.

- Follow the installation instructions.
  
    Ibex must be installed as a dynamic library. To this end, use the `enable-shared` option.
    A typical way to install ibex is therefore:

    `~/ibex$ ./waf configure --prefix=~/ibex --lp-lib=soplex --enable-shared`
    
    `~/ibex$ ./waf install`

OpenMesh
----------
- Download OpenMesh from [www.openmesh.org](www.openmesh.org).

    The current version has been tested with **Release 8.1** of OpenMesh.

- Follow the installation instructions.

Volga
----------
- Create a new workspace, e.g., `volga_ws`:
  
     `~$ mkdir -p volga_ws/src`
     
- Install the root folder `volga` of the Volga project under `volga_ws/src`. You must obtain 
  the following folder hierarchy:
  
    `~/volga_ws/src/volga/volga_core`  
    `~/volga_ws/src/volga/volga_examples`  
    `~/volga_ws/src/volga/volga_moveit`  

- Set the `pkgconfig` path of the external libraries `Ibex` and `OpenMesh`:

     `~$ export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:[path-to-ibex]/share/pkgconfig:[path-to-openmesh]/libdata/pkgconfig/`

- Build and install the `volga_core` and `volga_examples` in `volga_ws/src/`

     `~$ cd volga_ws`
     
     `~/volga_ws$ catkin config --install`
     
     `~/volga_ws$ catkin build volga_core volga_examples`
     
     `~/volga_ws$ source install/setup.bash`

- Add the path of the external libraries `Ibex` and `OpenMesh`:

     `~/volga_ws$ export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:[path-to-ibex]/lib:[path-to-openmesh]/lib`

Examples
===========

You can now try the following launch files and node:

- Rigid body path validation
    
    `~$ roslaunch volga_examples rigid_body_path.launch`
  
- Kinematic path validation:
    
    Note: this example requires the `universal_robot` package (UR10 model).
    
    `~$ roslaunch volga_examples kinematic_path.launch`

- Validated inverse kinematics example:
    
    `~$ rosrun volga_examples ik_test`
  
Integration in Moveit
=======================

This section assumes you have successfully followed the steps above for
installing the core packages `volga_core` and `volga_examples`.

It explains how to build and install the `volga_moveit` package,
which contains a plugin for Moveit.

Install & Build
--------------------
  
The `volga_moveit` package requires a recent release of Moveit than the one distributed with ROS Melodic. 

It has been developed and tested with `moveit` 1.1.0.

- Install the last version of Moveit by following the instructions here: [https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html)
  
    This should result in several packages installed in a separate workspace, say `moveit_ws`.
   
    The `volga_moveit` package has been tested with the following package configuration:
  
 
    | geometric_shapes --------- 0.7.0  
    | moveit ------------------- 1.1.0    
    | moveit_msgs -------------- 0.10.0    
    | moveit_tutorials --------- 0.1.0    
    | moveit_visual_tools ------ 3.5.2    
    | panda_moveit_config ------ 0.7.4    
    | rviz_visual_tool --------- 3.7.0    
  
     Note that you may need to also install dependencies such as:

     - `ros-melodic-rosparam-shortcuts`
     - `ros-melodic-joint-state-publisher-gui`
     - `ros-melodic-ros-pytest`

- Source the Moveit workspace:
 
    `~$ source moveit_ws/devel/setup.bash`

- Move to the Volga workspace:
    
    `~$ cd volga_ws`
    
- Build the package:

    `~/volga_ws$ catkin build volga_moveit`

- Re-source:
    
    `~/volga_ws$ source install/setup.bash`
        
- Check that the volga plugin has been added:

    `$ rospack plugins --attrib=plugin moveit_core | grep volga`

Running
-----------------

- Launch the demo:
    
    `$ roslaunch panda_moveit_config demo.launch pipeline:=volga`
    
- In the Panels menu, select ``Motion Planning``
- In the ``Scene Objects`` tab of the MotionPlanning window, click on ``Import From Text``
- Choose the file volga_ws/src/volga_moveit/scene/triangle.scene
- Move the robot goal state so that it has to avoid the triangle pane
- Click on ``Publish Scene``
- Under the ``Planning`` tab, click on ``plan''
