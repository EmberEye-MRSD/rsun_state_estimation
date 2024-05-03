# rsun_state_estimation

This repository hosts the codebase responsible for transforming the MSO odometry data, implementing the failsafes and publishing the data to mavros, which sends it to the FCU, as well as performing post-processing to analyze the drift error of the MSO odometry.

**Note**: The MSO codebase is a component of the Airlab stack and cannot be included in our repository.

**Rviz**

```
rviz -d odometry.rviz
```

This rviz file helps in visualizing the mso odometry and displays the imu and odom frame.


**Running the MSO FailSafe and Odometry Transform**

```
roslaunch rsun_state_estimation vio_bridge.launch
```


**Running Post Processing Node**

```
roslaunch rsun_state_estimation ground_truth_viz.launch
```
