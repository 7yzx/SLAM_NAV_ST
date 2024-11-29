**For robot_location Package**

roslaunch steer_mini_gazebo steer_mini_sim_sensors.launch

get ros topic 
```  
/ackermann_steering_controller/cmd_vel
/ackermann_steering_controller/odom
/clock
/gains/left_rear_joint/parameter_descriptions
/gains/left_rear_joint/parameter_updates
/gains/right_rear_joint/parameter_descriptions
/gains/right_rear_joint/parameter_updates
/gazebo/link_states
/gazebo/model_states
/gazebo/parameter_descriptions
/gazebo/parameter_updates
/gazebo/performance_metrics
/gazebo/set_link_state
/gazebo/set_model_state
/imu
/joint_states
/rosout
/rosout_agg
/scan
/tf
/tf_static
```

so this sim publish   
imu topic is /imu
odom topic is /ackermann_steering_controller/odom
