### Realsense D435i VSLAM in RTAB-Map  ###


## structure
```
launch
    │  
    ├──realsenseD435i.launch.py         (啟動realsense D435i Node)
    │  
    ├──rtabmap.launch.py                (啟動RTAB-Map)
    │  
    ├──drone_mapping.launch.py          (同時啟動realsene D435i & RTAB-Map 建圖模式)
    │  
    ├──drone_localization.launch.py     (同時啟動realsene D435i & RTAB-Map 定位模式)
    │  
    ├──test.launch.py                   (just for test the launch file)
    │  
    config
        │
        ├──defult.rviz                      (rviz2's config)
        │
        ├──rtab.rviz                        (rviz2  in rtab config)
```

## paramters

| **參數名稱**     | **default value** | **description**                       |
|--------------|-------------------|---------------------------------------|
| rtabmap_args |                   | If --delete_db_on_start/-d: 刪除原本地圖        |
| mode         |                   | If true: Launch in localization mode. |
| rtabmap_viz  | false             | Launch RTAB-Map UI (optional).        |
| rviz         | true              | Launch RVIZ (optional).               |

## usage

```
ros2 launch drone_vslam drone_mapping.launch.py rtabmap_args:="-d"
```

