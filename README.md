# duck_utility
* Docker utility
	* docker.sh
		* Run specific Docker image and auto commit when container has terminated.
* MAVROS utility
* ROSBAG utility
	* add_frame_id_pose_to_odom.py
		* For rosbag, frame_id adding and convert geometry_msgs/PoseStamped topic to nav_msgs/Odometry
		* Especially, rosbag generated from tum/euroc data with EVO package fits with this 
* RVIZ utility
	* img2rvizmarker.py
		* image to RVIZ marker
		* image file, scale, offset x/y configurable
		* example
			```bash
			python img2rvizmarker.py -f ./modified_airport_Cad.png -s 4.25 -y 16 -x -11
			```