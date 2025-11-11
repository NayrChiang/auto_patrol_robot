#!/bin/bash

echo "🛑 Stopping Gazebo..."

# Stop all Gazebo-related processes
pkill -f "ign gazebo" 2>/dev/null
pkill -f "ros2 launch" 2>/dev/null
killall ign-gazebo-server 2>/dev/null
killall ign-gazebo-gui 2>/dev/null

# Wait for processes to stop
sleep 1

# Force kill any remaining processes
ps aux | grep -E "(gazebo|ign.*gazebo)" | grep -v grep | awk '{print $2}' | xargs kill -9 2>/dev/null

echo "✅ Gazebo stopped"
