# Gazebo SITL Plugin #

<a href="https://scan.coverity.com/projects/01org-gazebo-sitl">
  <img alt="Coverity Scan Build Status"
       src="https://scan.coverity.com/projects/8893/badge.svg"/>
</a>

A ROS-independent Gazebo plugin for Ardupilot's SITL.

## Requirements ##
    * Python 2.7+ (to generate mavlink headers)
    * MavProxy (https://github.com/Dronecode/MAVProxy)
    * Ardupilot (https://github.com/ArduPilot/ardupilot)

## Build and Install ##

1. Make sure you have initialized and updated the Mavlink submodule at least
once with:

    ```
    git submodule update --init --recursive
    ```
2. Create a build folder, make and install using CMAKE as follows:

    ```
    mkdir build
    cd build
    cmake ..
    sudo make install
    cd -
    ```

3. Download Gazebo models if necessary:

    Gazebo automatically searches and downloads models referenced in the .world
    file that are not already present into GAZEBO_MODEL_PATH.
    However, in some cases the download might take a long
    time for no apparent reason. Download the Quadrotor model manually with:

    ```
    curl http://models.gazebosim.org/quadrotor/model.tar.gz | tar xvz -C GAZEBO_MODEL_PATH
    ```

## Run ##

1. Open a second terminal and run Ardupilot:

    ```
    ${ARDUPILOTDIR}/build/sitl/bin/arducopter --model x --defaults ${ARDUPILOTDIR}/Tools/autotest/default_params/copter.parm
    ```

2. In a third terminal, run Mavproxy:

    ```
    mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:15556
    ```

3. Open the Gazebo gzsitl_drone_target world file with the following command:

    ```
    gazebo ./world/gzsitl_drone_target.world --verbose
    ```

## Interaction ##

The Plugin first detects if the vehicle being simulated on SITL is Landed or
already Airborne.
 - If Landed, store the initial position as home, set flight mode as guided
   and request take-off.
 - If Airborne, request home position, and keep the current flight mode.

When Airborne, the vehicle follows the transparent sphere (target).

## Troubleshooting ##

Gazebo might fail to open if the needed models or plugins are not found. If
that happens, make sure the needed plugins have been successfully built and
installed into your shared libraries directory and that all the needed models
are present into GAZEBO_MODEL_PATH. The Gazebo --verbose terminal output is
useful for determining whether a plugin or a model has not not been found.

