# MICRO-ROS GAZEBO

This example describe how to establish the micro-ROS connection between Gazebo and ESP32

## Steps to follow

* Navigate to micro-ROS workspace (for example:- `microros_ws`) 
    * ```cd microros_ws/``` (or name of your workspace directory)

* Start and run the micro-ROS agent on your local system by executing following commands:

```bash
colcon build
source install/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```



* Open another terminal and source the ESP-IDF environment
    * ``` cd MARIO/firmware/4_microros_gazebo ```
    * ```$IDF_PATH/export.sh``` (or ```get_idf``` if you have set up the alias in your `.bashrc` file)
    
* Configure the micro-ROS settings using ```idf.py menuconfig```
    
    * Inside micro-ROS Settings Set up WiFi configuration (SSID and Password)
    * IP address of host PC in micro-ROS Agent IP (can be found by `ifconfig` command)
    * Make sure micro_ros port is same as given to the agent in previous command.

**Note** If it shows the error `ifconfig: command not found`, install the package `net-tools` by following command:

```bash
sudo apt install net-tools
```

* Build and Flash the code 
    * ```idf.py build```
    * ```idf.py -p PORT flash``` (For example: ```idf.py -p /dev/ttyUSB0 flash``` )



* In another terminal, Navigate to the ROS2 workspace and Start the Publisher by executing the relevant python script after sourcing ros2 in the environment
(Make sure you have previously built the folder by executing `colcon build` command with 4_simulation_gazebo in the src folder):

```bash
cd ros2_ws
source install/setup.bash
ros2 launch simulation_gazebo basic_gazebo.launch.py
```
This will open a gazebo simulation window.
Now we run kinematics scripts on the bot. Open a new terminal and run following commands.
```bash
cd ros2_ws
source install/setup.bash
ros2 run simulation_gazebo forward_kinematics.py
```

* Enter the angles on the publisher side and voila! See your manipulator moving!
