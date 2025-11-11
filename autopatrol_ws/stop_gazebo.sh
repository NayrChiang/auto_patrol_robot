#!/bin/bash

echo "🛑 停止 Gazebo..."

# 停止所有 Gazebo 相关进程
pkill -f "ign gazebo" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
killall ign-gazebo-server 2>/dev/null
killall ign-gazebo-gui 2>/dev/null

# 等待进程停止
sleep 1

# 强制停止任何残留进程
ps aux | grep -E "(gazebo|ign.*gazebo)" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null

echo "✅ Gazebo 已停止"
