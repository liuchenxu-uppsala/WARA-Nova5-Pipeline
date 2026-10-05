#!/bin/bash
set -e

CONTAINER_NAME="dobot_dev"
IMAGE_NAME="dobot_nova5:humble"

# 主机上用来和容器共享文件的目录(自己写的脚本放这里,容器里能直接看到,
# 不用再 docker cp)。第一次用之前记得在主机上先建好:
#   mkdir -p ~/dobot_shared
HOST_SHARED_DIR="$HOME/dobot_shared"
CONTAINER_SHARED_DIR="/dobot_ws/shared"
mkdir -p "$HOST_SHARED_DIR"

# 每次都重新授权图形界面转发
xhost +local:docker

# 判断容器是否已经存在
if docker ps -a --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
    # 存在:判断是不是在运行
    if docker ps --format '{{.Names}}' | grep -Eq "^${CONTAINER_NAME}\$"; then
        echo "容器已在运行,直接进入..."
        docker exec -it "$CONTAINER_NAME" bash
    else
        echo "容器已停止,重新启动并进入..."
        docker start -ai "$CONTAINER_NAME"
    fi
else
    # 不存在:第一次用 rocker 创建,带上共享目录挂载
    echo "容器不存在,首次创建..."
    rocker --x11 --user \
        --volume "$HOST_SHARED_DIR":"$CONTAINER_SHARED_DIR" \
        --name "$CONTAINER_NAME" "$IMAGE_NAME"
fi
