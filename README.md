#### Launch the test robot and SLAM_toolbox:
``` 
ros2 launch lunabot_bringup test_bot_navigation.launch.py
```

##### Wait for RViz to finish loading before launching Navigation2.

#### Launch Navigation2: 
```
ros2 launch nav2_bringup navigation_launch.py
```

##### There is no need to set the initial pose, it is already set as (0, 0). Select "Nav2 Goal" in RViz and choose where you'd like the robot to navigate to.