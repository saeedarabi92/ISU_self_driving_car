<div align="center">

# Iowa State University self driving car
</div>

<p align="center">
  <img width="200" height="200" src="images/ISU.jpg"> 
  <img width="200" height="170" src="images/IMG_6933.JPG">
    <img width="200" height="175" src="images/reactor.png">
</p>

 <!-- <p align="center">
  <img width="460" height="360" src="images/IMG_6933.JPG">
</p> -->

---

# TODO:

- [ ] Fix the battery issue of the car
- [x]   Setting up ROS and Pacmod related packages on the new Ubuntu laptop
- [ ]    Testing the drive by wire system of the car
  - [ ]    Driving the car with Joystick
  - [ ]    Testing the joystick commands and ensure their correctness (Acceleration/ Deceleration, steering)
  - [ ]    Checking the CAN messages and confirming their correctness

- [ ]   Setting up gazebo simulation
  - [ ]   Setting up a plane world gazebo simulation
  - [ ]   Spawning the car representative (simple ackerman steering model or the actual GEM polaris model)
  - [ ]   Testing the simulator by publishing the throttle and steering commands
  - [ ]   Adding GPS plugin in the Gazebo
  - [ ]   Adding IMU plugin in the Gazebo
  - [ ]   Sensor fusion for localization
  - [ ]   Adding the camera model
  - [ ]   Adding the Lidar model