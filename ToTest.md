需要在小车上进行的test文件，

## 11.17 需要测试

急需解决的问题

* [ ] 遇到的问题nav中那些参数到底怎么回事啊
* [ ] robot_location 为什么没有发布odom 到base_link



* [ ] robot_location  融合一下imu和odom，
* [ ] 以及使用odom的cartographer建图效果，
* [ ] 并且尝试在程序里写一个巡航的功能。

1. 首先需要建一个普通的功能包，比如

```python
cd Test_ws

catkin_makecd src

catkin_create_pkg gmapping_my_pkg std_msgs roscpp rospy sensor_msgs message_generation
```

参考的是[ekf_gmapping_steer_mini_sensors](Docker_noetic\neor_mini\mini_sim18_ws\src\mini_gmapping\launch\ekf_gmapping_steer_mini_sensors.launch)




## BUG记录

### move_base中

1. 

[ WARN] [1731850170.450399371]: Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.

https://answers.ros.org/question/188847/hydro-error-in-move_baselaunch/

meter_scoring : true



2

[ WARN] [1731850775.977229267]: Map update loop missed its desired rate of 10.0000Hz... the loop actually took 4.8501 seconds

cpu 性能不够，地图太大了。
