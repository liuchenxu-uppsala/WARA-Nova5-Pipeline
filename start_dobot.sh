#!/bin/bash
set -e

CONTAINER_NAME="dobot_dev"
IMAGE_NAME="dobot_nova5:humble"

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
    # 不存在:第一次用 rocker 创建
    echo "容器不存在,首次创建..."
    rocker --x11 --user --name "$CONTAINER_NAME" "$IMAGE_NAME"
fi
