FROM ghcr.io/ros-navigation/nav2_docker:rolling-nightly

WORKDIR /root/nav2_ws

RUN cd src && \
    git clone https://github.com/CihatAltiparmak/frenet_ilqr_controller.git

RUN wget https://gist.githubusercontent.com/CihatAltiparmak/7171fbb514287501ce91e9c45c69dab2/raw/b24ba47669951f26543e294993a88e847480e582/nav2_param_frenet_ilqr_controller_demo.yaml -O src/navigation2/nav2_bringup/params/nav2_param_frenet_ilqr_controller_demo.yaml

RUN . /opt/ros/rolling/setup.sh && \
    colcon build --packages-select nav2_bringup nav2_frenet_ilqr_controller frenet_trajectory_planner ilqr_trajectory_tracker --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN touch /entrypoint.sh && \
    echo "#!/bin/bash" >> /entrypoint.sh && \
    echo "set -e" >> /entrypoint.sh && \
    echo "source /opt/ros/rolling/setup.bash" >> /entrypoint.sh && \
    echo "source /root/nav2_ws/install/setup.bash" >> /entrypoint.sh && \
    echo 'exec "$@"' >> /entrypoint.sh && \
    chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]