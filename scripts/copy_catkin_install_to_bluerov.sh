#! /bin/bash

LOCAL_CATKIN_WORKSPACE="/home/${USER}/Dev/ros-catkin-workspace"
LOCAL_CATKIN_INSTALL_SPACE="${LOCAL_CATKIN_WORKSPACE}/install"
ROBOT_CATKIN_WORKSPACE="/home/bluerov/catkin-workspace"
ROBOT_WIFI_NET="bluerov-companion-net"
ROBOT_WIFI_PASSWORD="BlueROV2"

TARBALL_NAME="$(date -u +%a-%b-%d_%I-%M%p)-install.tar.gz"

ROBOT_TARBALL_DESTINATION="/tmp"

CURRENT_WIFI=$(iwgetid -r)
CURRENT_DIR=$(pwd)

echo "Building..."
cd ${LOCAL_CATKIN_WORKSPACE} && source install/setup.bash && catkin_make && catkin_make install

if [ $? -ne 0 ]
then
    exit $?
fi

cd ${CURRENT_DIR}

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
bash ${DIR}/connect_to_robot_wifi.sh


if [ $? -ne 0 ]
then
    exit $?
fi

set -eo pipefail
echo "Making install tarball..."
cd ${LOCAL_CATKIN_WORKSPACE}
tar -czf ${TARBALL_NAME} install/
cd ${CURRENT_DIR}
echo "Successfully created tarball."

echo "Installing tarball..."
scp ${LOCAL_CATKIN_WORKSPACE}/${TARBALL_NAME} bluerov@bluerov:${ROBOT_TARBALL_DESTINATION}
ssh bluerov@bluerov "rm -rf ${ROBOT_CATKIN_WORKSPACE}/install"
ssh bluerov@bluerov "cd ${ROBOT_TARBALL_DESTINATION} && tar -xzf ${TARBALL_NAME} && mv install ${ROBOT_CATKIN_WORKSPACE}/install"
echo "New install space copied!"

# TODO have the app directly in the package
scp ${LOCAL_CATKIN_WORKSPACE}/GeminiMultibeamSonar/bin/GeminiSDKConsoleApp bluerov@bluerov:${ROBOT_CATKIN_WORKSPACE}/install/share/gemini_multibeam_ros/

echo "Removing local copy of tarball"
rm ${LOCAL_CATKIN_WORKSPACE}/${TARBALL_NAME}

if [ "$1" == "reconnect" ]; then
    if [ "${CURRENT_WIFI}" != "${ROBOT_WIFI_NET}" ]
    then
        echo "Reconnecting to original wifi."
        if [ "${CURRENT_WIFI}" == "samnet" ]
        then
            nmcli dev wifi connect ${CURRENT_WIFI} password welcometosamnet
        else
            nmcli dev wifi connect ${CURRENT_WIFI}
        fi
    fi
fi

notify-send 'Finished compiling and installing ROS code' -u low
