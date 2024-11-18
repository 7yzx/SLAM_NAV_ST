需要在小车上进行的test文件，


## 11.18 测试

* [ ] 使用别的融合方式，看看效果https://blog.csdn.net/KYJL888/article/details/113315100   https://www.ncnynl.com/archives/201708/1909.html
* [ ] 使用3d雷达看看
* [ ] 解决map更新的方法
  * [ ] 在map中把分辨率调大一些。
  * [ ] 





## 11.17 需要测试

急需解决的问题

* [ ] 遇到的问题nav中那些参数到底怎么回事啊
* [ ] robot_location 为什么没有发布odom 到base_link
* [ ] robot_location  融合一下imu和odom，这个用不了就用https://blog.csdn.net/KYJL888/article/details/113315100   https://www.ncnynl.com/archives/201708/1909.html
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

不错的解决方案
https://blog.csdn.net/YiYeZhiNian/article/details/122351506

[ WARN] [1731850170.450399371]: Trajectory Rollout planner initialized with param meter_scoring not set. Set it to true to make your settings robust against changes of costmap resolution.

https://answers.ros.org/question/188847/hydro-error-in-move_baselaunch/

meter_scoring : true


#### 导航问题
 WARN] [1731939467.152310401]: The robot's start position is off the global costmap. Planning will always fail, are you sure the robot has been properly localized?

https://blog.csdn.net/markchalse/article/details/107032550

#### map更新

[ WARN] [1731850775.977229267]: Map update loop missed its desired rate of 10.0000Hz... the loop actually took 4.8501 seconds

cpu 性能不够，地图太大了。




#### 融合ekf

问题一：odom斜方差
https://blog.csdn.net/datase/article/details/83095458

问题二：重复发布

https://blog.csdn.net/FRIGIDWINTER/article/details/126291629