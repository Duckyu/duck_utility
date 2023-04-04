# duck_utility
* Docker utility
	* docker.sh
		* Run specific Docker image and auto commit when container has terminated.
* MAVROS utility
* RVIZ utility
	* img2rvizmarker.py
		* image to RVIZ marker
		* image file, scale, offset x/y configurable
		* example
			```bash
			python img2rvizmarker.py -f ./modified_airport_Cad.png -s 4.25 -y 16 -x -11
			```