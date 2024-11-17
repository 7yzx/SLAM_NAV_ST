需要注意的tf变换关系


## 在VLP16 liosam中


```launch
   <node name="laser_to_base_link" pkg="tf" type="static_transform_publisher" args="0.0 0.0 0.135 0 0 0 base_link laser_link 40 " />
   <node name="imu_to_base_link" pkg="tf" type="static_transform_publisher" args="-0.10 0.0 0.02 0 0 0 base_link imu_link 40 " />
   <node name="camera_to_base_link" pkg="tf" type="static_transform_publisher" args="0.12 0.0 0.12 0 0 0 base_link camera_link 40 " />
```
