```
# 终端1：启动环境
roscore

# 终端2：启动gazebo环境
roslaunch racecar_gazebo racecar_runway_navigation.launch

# 终端3：启动路径跟踪
rosrun racecar_gazebo path_pursuit.py

# 终端4：运行目标点导航
rosrun racecar_gazebo simple_goals_with_monitor.py
```