需要在小车上进行的test文件，

## 11.17 需要测试

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
