FROM osrf/ros:foxy-desktop

SHELL ["/bin/bash", "-c"]

WORKDIR /acciona_challenge

COPY foxy foxy/
RUN apt-get -y update
RUN apt-get install -y python3-pip
RUN sudo apt-get install ros-foxy-turtle-tf2-py ros-foxy-tf2-tools ros-foxy-tf-transformations -y
RUN pip3 install transforms3d && \
    cd foxy/dev_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build --symlink-install

# COPY docker_foxy/ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]