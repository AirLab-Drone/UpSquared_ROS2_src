# UpSquaredWorkSpace


### aruco_detect
```
pip install pyudev
```

### flight_control
```
pip install opencv-python opencv-contrib-python scipy cv_bridge
```
mavros  
```
sudo apt install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```


## 環境
<table>
<tbody>
    <tr>
        <td>作業系統</td>
        <td>         
            <a href="https://releases.ubuntu.com/jammy/">Ubuntu 22.04</a>
        </td>
        <td>
            <img alt="Static Badge" src="https://img.shields.io/badge/build-passing-green">
        </td>
    </tr>
    <tr>
        <td>ROS</td>
        <td>         
            <a href="https://docs.ros.org/en/humble/index.html">Humble</a>
        </td>
    </tr>
</table>

## 依賴
- 安裝 [rtabmap-ros](https://github.com/introlab/rtabmap_ros)
    (測試安裝 from source)

- 安裝 [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

- 安裝 [Octomap](https://github.com/OctoMap)
    ``` 
    sudo apt-get install ros-humble-octomap*
    ```

- 安裝 [Apriltag](https://github.com/AprilRobotics/apriltag)
    ``` 
    sudo apt-get install ros-humble-apriltag*
    ```

### 也許需要安裝
**Gazebo classic**
```
sudo apt install ros-humble-gazebo-ros-pkgs
```




