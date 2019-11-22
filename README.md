BEFORE ANYTHING:
source devel/setup.sh

LAUNCH PROJECT:
roslaunch ivr_assignment spawn.launch

RUN CODE:
source temp-python/bin/activate
rosrun ivr_assignment image1.py

MOVE ROBOT:
rostopic pub -1 /robot/joint1_position_controller/command std_msgs/Float64 “data: 1.0”

PRODUCE GRAPHS FOR TARGET ESTIMATES:
- for each one, run the command
- click on the arrow button to set axes (choose around 50 seconds)
- save as an image
rqt_plot /target_x_estimate/data /target/x_position_controller/command/data
rqt_plot /target_y_estimate/data /target/y_position_controller/command/data
rqt_plot /target_z_estimate/data /target/z_position_controller/command/data
rqt_plot /target_z_estimate2/data /target/z_position_controller/command/data
compare the z estimate from image 1 and image 2:
rqt_plot /target_z_estimate/data /target_z_estimate2/data
