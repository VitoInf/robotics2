#!/bin/bash
CONTAINER_NAME=ros2_hmbl_container
USER_NAME=test_user

echo "==================================="
echo "  Data Logger Launcher"
echo "==================================="
echo ""
echo "Choose robot to log:"
echo "  1) Leader (default)"
echo "  2) Left Follower"
echo "  3) Right Follower"
echo ""
read -p "Enter choice [1-3] (default: 1): " choice

case $choice in
    2)
        ROBOT_NAME="left_follower"
        echo "Starting logger for LEFT FOLLOWER..."
        ;;
    3)
        ROBOT_NAME="right_follower"
        echo "Starting logger for RIGHT FOLLOWER..."
        ;;
    *)
        ROBOT_NAME=""
        echo "Starting logger for LEADER..."
        ;;
esac

# Execute data logger in running container
if [ -z "$ROBOT_NAME" ]; then
    # Leader
    docker exec -it $CONTAINER_NAME bash -c "
        source /opt/ros/humble/setup.bash
        source /home/$USER_NAME/multirobot_ws/install/setup.bash
        ros2 run multirobot_sim data_logger --ros-args -p plot_live:=true
    "
else
    # Follower
    docker exec -it $CONTAINER_NAME bash -c "
        source /opt/ros/humble/setup.bash
        source /home/$USER_NAME/multirobot_ws/install/setup.bash
        ros2 run multirobot_sim data_logger --ros-args \
            -p robot_name:=$ROBOT_NAME \
            -p plot_live:=true \
            -p log_file:=${ROBOT_NAME}_log_\$(date +%Y%m%d_%H%M%S).npz
    "
fi
