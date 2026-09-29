FROM osrf/ros:humble-desktop

# 基础工具 + 缺失的 ros2_control 相关包
RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros2controlcli \
    ros-humble-joint-trajectory-controller \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# 初始化 rosdep(镜像里通常已经init过,这里做一次update保险)
RUN rosdep update

# 工作空间放在 /dobot_ws,不放在 /root 下面,普通用户也能访问
WORKDIR /dobot_ws/src

RUN git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3.git

WORKDIR /dobot_ws

RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y --skip-keys="warehouse_ros_mongo"

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

# 让所有用户都能读写这个目录(rocker --user 会用你主机的用户身份进容器)
RUN chmod -R a+rwX /dobot_ws

# 全局 bashrc,任何用户进来都会自动 source,并固定机型
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /dobot_ws/install/setup.bash" >> /etc/bash.bashrc && \
    echo "export DOBOT_TYPE=nova5" >> /etc/bash.bashrc

CMD ["/bin/bash"]
