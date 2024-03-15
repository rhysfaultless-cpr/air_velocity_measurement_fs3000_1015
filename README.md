# air_velocity_measurement_fs3000_1015

<center><img src="/readme_assets/clearpath_robotics_pacs_with_sparkfun_fs3000_1015_kit.png" width="300"/></center>

> [!IMPORTANT]  
> This branch is for [ROS 2 Humble](https://docs.ros.org/en/humble/index.html).

> [!NOTE]  
> This driver has been tested with a [Sparkfun FS3000-1015](https://www.sparkfun.com/products/18768) connected to a SparkFun Artemis development board.
> As of writing in March 2024, this is available from SparkFun and Digi-Key [KIT-21310](https://www.sparkfun.com/products/21310?_ga=2.32017463.864881375.1710459625-374013097.1696440917)

<br />

## Installation

1.  Create a workspace and src directory, `~/ros2_ws/src/`.
2.  Clone this git repository to the directory `~/ros2_ws/src/`.
    You now have a directory structure of:

    ```bash
    ~/ros2_ws/
        └── src/
            └── air_velocity_measurement_fs3000_1015/
                ├── debian/
                |   ├── ...
                |   └── ...
                ├── air_velocity_measurement/
                |   ├── ...
                |   └── ...
                ├── air_velocity_measurement.ino
                └── README.md
    ```

    > [!NOTE]  
    > Files related to the ROS 2 package are all within the folder `air_velocity_measurement/`
    > The other files and folders are used for system setup. 


3.  Install the udev rule:
    ```
    sudo cp ~/ros2_ws/src/air_velocity_measurement_fs3000_1015/debian/udev /etc/udev/rules.d/43-sparkfun-fs3000-1015.rules
    sudo udevadm control --reload-rules
    sudo udevadm trigger
    ```
4.  Connect the hardware: 

    - Sensor → Development Board wit Qwiic cable
    - Development Board → Computer with USB cable

5.  Open the Arduino IDE.
    
    - Configure the IDE to accept the SparkFun Artemis development board, and the SparkFun_FS3000_Arduino_Library.
      This process is described in SparkFun's documentation.
    - Add the contents of this repository's file `air_velocity_measurement.ino` to your Arduino sketch.
      This file has the same content as SparkFun's example, but making configuring the sketch for metric units and the 1015 variant of the sensor.
    - Build and flash the sketch to the SparkFun Artemis development board

6.  Build the workspace:
    ```
    cd ~/ros2_ws
    colcon build
    ```

7.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```

<br />

## Usage

### Running the Driver

1.  Open a new terminal.
2.  Source the workspace:
    ```
    source ~/ros2_ws/install/setup.bash
    ```
3.  Start the node with either:

    ```
    ros2 launch air_velocity_measurement air_velocity_measurement_launch.py
    ```

    or

    ```
    ros2 run air_velocity_measurement air_velocity_measurement
    ```

    > [!NOTE]  
    > `ros2 run` may have issues if the sensor is connected as a device other than the default `/dev/ttyUSB0`.
    > The launch file includes line to change this parameter.

<br />


### Topics and Services

This will create a topic called `/air_velocity_measurement`.
The topic publishes an integer, which represents the sensor's reported air speed in cm/s
