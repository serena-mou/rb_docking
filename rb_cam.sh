#!/bin/bash
screen -dmS core -s /bin/bash 
screen -S core -p 0 -X stuff 'source ~/.bashrc\n' 
screen -S core -p 0 -X stuff 'source /opt/ros/melodic/setup.bash\n' 
screen -S core -p 0 -X stuff 'roscore\n' 


screen -dmS usb_cam -s /bin/bash
sleep 2
screen -S usb_cam -p 0 -X stuff 'cd ~/docking/docking_code\n'
screen -S usb_cam -p 0 -X stuff 'source ~/.bashrc\n'
screen -S usb_cam -p 0 -X stuff 'source ~/catkin_ws/devel/setup.bash\n' 
screen -S usb_cam -p 0 -X stuff 'python3 usb_cam.py\n' 

screen -dmS dock -s /bin/bash 
screen -S dock -p 0 -X stuff 'cd ~/docking/docking_code\n' 
screen -S dock -p 0 -X stuff 'source ~/.bashrc\n' 
screen -S dock -p 0 -X stuff 'source ~/catkin_ws/devel/setup.bash\n' 
screen -S dock -p 0 -X stuff 'python3 rb_docking.py\n' 

screen -dmS serial -s /bin/bash 
screen -S serial -p 0 -X stuff 'cd ~/docking/docking_code\n' 
screen -S serial -p 0 -X stuff 'source ~/.bashrc\n' 
screen -S serial -p 0 -X stuff 'source ~/catkin_ws/devel/setup.bash\n' 
screen -S serial -p 0 -X stuff 'python3 rb_angle_serial.py\n'
#
#screen -dmS model
#screen -S model -p 0 -X stuff 'source ~/.bashrc\n'
#screen -S model -p 0 -X stuff 'cd ~/coral-code/classifier/\n'
#screen -S model -p 0 -X stuff 'python3 ros_classify.py\n'
#screen -d -m -S avcam bash -c 'roslaunch pointgrey_avcam_driver avcam.launch avcam_serial:=14110875' &
#screen -d -m -S classify bash -c 'python3 ~/coral-code/classifier/ros_classify.py'
