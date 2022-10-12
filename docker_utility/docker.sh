#! /bin/bash

## Obtain authority for x-server
xhost local:root

## Args
export SELECTED_IMAGE=duckyu/uavmvs:ros-melodic
export WS_LOCATION="/home/duckling/ws/unROS/uavmvs_ws:/root/uavmvs_ws:rw"
#export DATA_LOCATION="/home/duckling/ws/unROS/uavmvs_ws:/root/uavmvs_ws:rw"

## Docker run %% Aware of -rm option. It may remove the latest container 
sudo docker run -it --privileged -p="8888:8888" -p "6006:6006" -v=${WS_LOCATION} -v "/var/lib/dbus/machine-id:/var/lib/dbus/machine-id" -e "DISPLAY=${DISPLAY}" -v="/tmp/.X11-unix:/tmp/.X11-unix:rw" --shm-size=32g --gpus all -w /root --net=host ${SELECTED_IMAGE} bash
echo "Container terminated!"

## Detect latest container of target image
sudo docker ps -a >> image_history.txt

export IMAGE=$(awk -v selected_image=${SELECTED_IMAGE} '$2 == selected_image {print $2;exit}' image_history.txt)
export LAST_ID=$(awk -v selected_image=${SELECTED_IMAGE} '$2 == selected_image {print $1;exit}' image_history.txt)

rm image_history.txt
echo "Latest container detected!!"

## Auto commit the detected container
if [ ${IMAGE} == ${SELECTED_IMAGE} ] ; then
	echo "Attempt commit ${LAST_ID} ${IMAGE}"
	sudo docker commit ${LAST_ID} ${SELECTED_IMAGE}
	echo "Commit complete!!!"
else
	echo "IMAGE: ${IMAGE}"
	echo "No container running!!!!"
fi

## Clear authority for x-server
xhost -local:root
