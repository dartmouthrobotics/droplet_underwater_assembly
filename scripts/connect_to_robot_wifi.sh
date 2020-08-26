ROBOT_WIFI_NET="bluerov-companion-net"
ROBOT_WIFI_PASSWORD="BlueROV2"

CURRENT_WIFI=$(iwgetid -r)

if [ "${CURRENT_WIFI}" != "${ROBOT_WIFI_NET}" ]
then
    echo "Connecting to robot..."
    nmcli dev wifi | grep ${ROBOT_WIFI_NET}

    if [ $? -eq 0 ];
    then
        :
    else
        echo "Robot network not found. Rescanning..."
        nmcli dev wifi rescan
        sleep 15

        nmcli dev wifi | grep ${ROBOT_WIFI_NET}
        if [ $? -eq 0 ];
        then
            :
        else
            echo "Robot wifi network not found. Terminating."
            exit 1
        fi
    fi

    nmcli dev wifi connect ${ROBOT_WIFI_NET} password ${ROBOT_WIFI_PASSWORD}

    if [ $? -eq 0 ]; then
        echo "Connected to robot."
    fi
else
    echo "Already connected to robot wifi."
fi

if [ $(iwgetid -r) != "${ROBOT_WIFI_NET}" ]
then
    echo "Failed to connect to robot. Terminating."
    exit 1
fi
