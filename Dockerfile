FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_snake

COPY dev_ws dev_ws/
RUN apt-get -y update
RUN apt-get install -y python3-pip
RUN apt-get install -y x11-apps
RUN sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations -y
RUN pip3 install transforms3d && \
    cd dev_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

# COPY ros_entrypoint.sh ros_entrypoint.sh
# RUN  chmod 755 ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]