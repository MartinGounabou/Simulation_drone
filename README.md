
# ROS-Quadcopter-Simulation

# Installation #


cd ~/danger_ws/src
catkin_create_pkg fly_bot rospy std_msgs


```
cd ~/danger_ws
catkin_make
```

lancer  gazebo
```
roslaunch fly_bot Kwad_gazebo.launch

```


```
rostopic pub -1 /Kwad/joint_motor_controller/command std_msgs/Float64MultiArray "data: [50, -50, 50, -50]"

```

pour lancer la regulation


```
rosrun fly_bot control.py

```

resultat

[Video](https://youtu.be/50IAZ0us7ZY)

