# python required
opencv-python  
opencv-contrib-python  
scipy  
cv_bridge  
mavros  
```
sudo apt install ros-humble-mavros ros-humble-mavros-extras
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod a+x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

# 座標係
## 飛控座標
x 軸: 前進為正，後退為負  
y 軸: 左移為正，右移為負  
z 軸: 上升為正，下降為負  

## marker 相機
marker相對於相機的座標  
x 軸: 向右為正，向左為負  
y 軸: 向後為正，向前為負  
z 軸: 向上為正，向下為負  

## 啟動MAVROS
```
sudo chmod 777 /dev/ttyACM0
ros2 launch mavros apm.launch fcu_url:=/dev/ttyACM0
```
### 修改發布頻率
```
ros2 run mavros mav sys message-interval --id 24 --rate 100
ros2 run mavros mav sys message-interval --id 33 --rate 100
ros2 run mavros mav sys message-interval --id 132 --rate 100
```
