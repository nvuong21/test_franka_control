# netft

This is a ROS metapackage for the ATI NET/FT box that is used to connect to ATI F/T sensor.

## Compatibility

The `master` branch is compatible with:

* ROS Hydro
* ROS Indigo
* ROS Kinetic

## Requirements

Install `curl` manually:

```
sudo apt-get install libcurl4-openssl-dev
```

## Usage

Use the ROS node in the package as a stand-alone node for publishing F/T information. E.g.:

```
rosrun netft_control netft_node --address 192.168.1.1
```

The NET/FT box defaults to 192.168.1.1 as its ip address but this can be changed using the browser-based interface.

See a more detailed tutorial here: http://wiki.ros.org/netft_rdt_driver/Tutorials/RunningNetFTNode

### ros-control Hardware Interface

For applications that require to read from the netft within ros-control, there is an implementation that uses a `hardware_interface::ForceTorqueSensorInterface` and a `force_torque_sensor_controller::ForceTorqueSensorController`.

For this, there are two convenient launch files:
```
roslaunch netft_control netft_ros_driver.launch ip:=192.168.1.1
roslaunch netft_control ros_controllers.launch
```
