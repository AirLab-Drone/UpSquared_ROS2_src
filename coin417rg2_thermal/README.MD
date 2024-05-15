
This is for thermal camera model CION417RG2
ubuntu 22.04 usb3.0 SDK

----


It look like:

![](https://github.com/AirLab-Drone/UpSquared_ROS2_src/blob/main/coin417rg2_thermal/doc/thermalInRos2Node.png)

![](https://github.com/AirLab-Drone/UpSquared_ROS2_src/blob/main/coin417rg2_thermal/doc/thermal_img.png)

### build
In youe work space folder
```
colcon build --packages-select coin417rg2_thermal
```


### Run with ros2 node
```
ros2 run coin417rg2_thermal COIN417RG2_ros2_node
```