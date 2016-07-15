# Gazebo Sitl Plugin #

A ROS-independent Gazebo plugin for Ardupilot's SITL.

## Requirements ##
    * Python 2.7+ (to generate mavlink headers)
    * MavProxy (https://github.com/Dronecode/MAVProxy)
    * Ardupilot (https://github.com/ArduPilot/ardupilot)

## Build and Install ##

1. Make sure you have initialized and updated the Mavlink submodule at least
once with:

    ```
    git submodule init
    git submodule update
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
    file that are not already present in ~/.gazebo/models or in
    GAZEBO_MODEL_PATH. However, in some cases the download might take a long
    time for no apparent reason. Download the Quadrotor model manually with:

    ```
    mkdir -p ~/.gazebo/models
    curl http://models.gazebosim.org/quadrotor/model.tar.gz | tar xvz -C ~/.gazebo/models
    ```

## Run ##

1. Open a second terminal and run Ardupilot:

    ```
    ${ARDUPILOTDIR}/build/sitl/bin/arducopter-quad --model x
    ```

2. In a third terminal, run Mavproxy:

    ```
    mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14556 --streamrate -1
    ```

    Load the Parameters on Mavproxy Prompt:

    ```
    param load ./parameters/gzsitl.parm
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

## Throubleshooting ##

Gazebo might fail to open if the needed models or plugins are not found. If
that happens, make sure the needed plugins have been successfully built and
installed into your shared libraries directory and that all the needed models
are present into ~/.gazebo/models. The Gazebo --verbose terminal output is
useful for determining wether a plugin or a models has not not been found.

