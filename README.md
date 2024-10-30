# UpSquaredWorkSpace


### aruco_detect
```
pip install pyudev
```
相機權限
1. 編輯 rc.local 文件（如果沒有，則創建一個）：
```
sudo nano /etc/rc.local
```
2. 在文件中添加以下內容，將 chmod 指令放在 exit 0 之前：
```
#!/bin/bash
chmod 777 /dev/video*
exit 0
```

3. 確保 rc.local 文件具有可執行權限：

```
sudo chmod +x /etc/rc.local
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




