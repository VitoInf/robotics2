#!/bin/bash
CONTAINER_NAME=ros2_hmbl_container
USER_NAME=test_user

echo "==================================="
echo "  Robot Analysis Launcher"
echo "==================================="
echo ""
echo "Choose robot to analyze:"
echo "  1) Leader (default)"
echo "  2) Left Follower"
echo "  3) Right Follower"
echo ""
read -p "Enter choice [1-3] (default: 1): " choice

case $choice in
    2)
        ROBOT_NAME="left_follower"
        echo "Starting analysis for LEFT FOLLOWER..."
        ;;
    3)
        ROBOT_NAME="right_follower"
        echo "Starting analysis for RIGHT FOLLOWER..."
        ;;
    *)
        ROBOT_NAME=""
        echo "Starting analysis for LEADER..."
        ;;
esac

# Trajectory generator in background (always for leader)
docker exec -d $CONTAINER_NAME bash -c "
    source /opt/ros/humble/setup.bash
    source /home/$USER_NAME/multirobot_ws/install/setup.bash
    ros2 run multirobot_sim trajectory_generator
"

# Logger in foreground (for selected robot)
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
