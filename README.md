# UpSquaredWorkSpace


### 相機權限 (RGB and thermal camera)
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

------ 


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




