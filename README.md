# Wall-Following-Robot
A Wall follower bot is a robot which follow the wall of the obstacle without colloiding with any of the obstacle.
We have build one differential drive robot which have ultrasonic sensor mounted on it along its front side, right and left hand side of the robot.
This robot will first avoid obstacle using the front sensor and will maintain the constant distance from the wall using corresponding sensors.
So the basic control that try the robot to follow the wall is PID ALGORITHM which applied on the error or deviation of the robot from its constant reference distance.
