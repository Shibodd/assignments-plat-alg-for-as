* Assignment #2 Kalman filter and extended kalman filter:
    - Objectives: This project is divided into two parts:
         + 1)  Implement an extended kalman filter to estimate the state of an object using lidar and radar measurements. A rosbag is provided for the experiments. The executable generates a file with the trajectory followed by the vehicle. Files to modify: kalman_filter.cpp, fusionEKF.cpp, tools.cpp (See the presentation)
                * write a short report explaining the achieved results (run plotter.py to observe the trajectory estimated). Please provide an explanation of the results
                    * In the report it is very positively scored the presentation of the EKF in different scenarios. 
                        - For instance: what happens with the trajectory if the motion noise is very high? and what if the sensor noise is very very small?. In a nutshell, play with the kalman filter variables and discuss the results.

        
* OS requirements:
    + ROS (the installation command depends on the Linux distribution)
    Example: sudo apt install ros-melodic-desktop-full (this one applies to Ubuntu 18)

* Instructions to compile the code:
    
    + EKF:
        Open one terminal and run the following command: roscore
        To compile the code run this command in a different terminal: catkin_make 
	    The executable can be found in: ./devel/lib/ekf/ekf_node (run this command in a terminal to start the execution)
        As soon as you launch the executable you can start the simulation. To do so, open another terminal and write:
            rosbag play 2022-07-26-12-45-29.bag
            This file is in the folder 'EKF/data'
        The executable generates a file called 'res.txt'
        To create the plot just write 'python3 plotter.py' (notice that the program only works if the KF/EKF has been correctly implemented)

* Note the rosbag file (2022-07-26-12-45-29.bag) contains three different type of messages:
    + LiDAR messages: nav_msgs::Odometry. This message contains:
        Header header
        string child_frame_id
        geometry_msgs/PoseWithCovariance pose
        geometry_msgs/TwistWithCovariance twist


    + Radar messages: ekf::RadarMsg. Custom message that contains:
        int32 id
        int32 timestamp
        float32 rho
        float32 theta
        float32 rho_dot

    + Ground truth messages: nav_msgs::Odometry
