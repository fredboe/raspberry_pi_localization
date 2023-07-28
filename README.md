# raspberry_pi_localization
In this project I want to test the performance of a kalman filter.
To achieve that goal i built a robot that is controlled by the raspberry pi. 
The robot has two (control-)modes:

1. Controlled by a joystick. While in this mode the robot drives by the commands of the joystick. It creates a track that is based on gps and imu data (and in the end updated by a kalman filter).
2. Follow another track. In this mode the robot uses gps and imu data to follow a track.

My general plan consisted of two steps. In the first one wanted to create a track with the joystick-mode. 
Then in the second step I wanted to see how close the robot would follow the track that was created in step 1.

### Authors
Frederik Böcker