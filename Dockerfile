FROM ghcr.io/ros-navigation/nav2_docker:humble-nightly

WORKDIR /root/nav2_ws

RUN cd src && \
    git clone https://github.com/CihatAltiparmak/frenet_ilqr_controller.git -b humble

<<<<<<< HEAD
RUN wget https://gist.githubusercontent.com/CihatAltiparmak/b7fb6000309beb057d354fa4ac4a8b9a/raw/9080e04524ddaa35aad36052a403f3f27b85d21f/nav2_param_frenet_ilqr_controller_demo_humble.yaml -O src/navigation2/nav2_bringup/params/nav2_param_frenet_ilqr_controller_demo.yaml
=======
RUN wget https://gist.githubusercontent.com/CihatAltiparmak/7171fbb514287501ce91e9c45c69dab2/raw/65927e0f29ab3c266701913766f5f917ce6bbb30/nav2_param_frenet_ilqr_controller_demo.yaml -O src/navigation2/nav2_bringup/params/nav2_param_frenet_ilqr_controller_demo.yaml
>>>>>>> dbbecab (Add constraints policy to eliminate trajectories that does not obey (#79))

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select nav2_bringup nav2_frenet_ilqr_controller frenet_trajectory_planner ilqr_trajectory_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN apt update && \
    apt install ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-navigation2 -y

RUN touch /entrypoint.sh && \
    echo "#!/bin/bash" >> /entrypoint.sh && \
    echo "set -e" >> /entrypoint.sh && \
    echo "source /opt/ros/humble/setup.bash" >> /entrypoint.sh && \
    echo "source /root/nav2_ws/install/setup.bash" >> /entrypoint.sh && \
    echo "export TURTLEBOT3_MODEL=waffle" >> /entrypoint.sh && \
    echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]