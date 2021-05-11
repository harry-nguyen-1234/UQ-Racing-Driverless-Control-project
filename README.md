# UQ Racing Driverless Control project.

Project to control car given known trajectory. <br/>
Technologies: ROS, C++, Python, Linux (Ubuntu)

This image shows the calculated curvature from which to control the car's velocity.
![Trajectory with curvature](https://github.com/harry-nguyen-1234/UQ-Racing-Driverless-Control-project/blob/master/curvature%20map.png)

This image shows different speed profiles used to control the car velocity.
![Speed profiles](https://github.com/harry-nguyen-1234/UQ-Racing-Driverless-Control-project/blob/master/speed_profiles.png)

This gif shows the controller in action.
One lookahead is used for steering control, and another is used for speed control/accounting for braking distance.
![Car controller in action](https://github.com/harry-nguyen-1234/UQ-Racing-Driverless-Control-project/blob/master/variable_speed_control.gif)
