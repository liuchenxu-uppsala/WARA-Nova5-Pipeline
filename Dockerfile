FROM osrf/ros:humble-desktop

# 基础工具 + 缺失的 ros2_control 相关包
# 2026-10-01 更新:明确补上 joint-state-broadcaster(之前依赖 ros2-controllers 这个
# metapackage 间接带入,结果在旧镜像里实际没装上;这次显式列出,不再依赖隐式传递依赖)
RUN apt-get update && apt-get install -y \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros2controlcli \
    ros-humble-joint-trajectory-controller \
    ros-humble-joint-state-broadcaster \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

RUN rosdep update

# 工作空间放在 /dobot_ws,不放在 /root 下面,普通用户也能访问
WORKDIR /dobot_ws/src
RUN git clone https://github.com/Dobot-Arm/DOBOT_6Axis_ROS2_V3.git

WORKDIR /dobot_ws

RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y --skip-keys="warehouse_ros_mongo"

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

RUN chmod -R a+rwX /dobot_ws

# 全局 bashrc,任何用户进来都会自动 source,并固定机型
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /dobot_ws/install/setup.bash" >> /etc/bash.bashrc && \
    echo "export DOBOT_TYPE=nova5" >> /etc/bash.bashrc

CMD ["/bin/bash"]
