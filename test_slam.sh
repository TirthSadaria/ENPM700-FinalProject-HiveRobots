#!/bin/bash
echo "=== Testing SLAM Setup ==="

echo "1. Checking odometry..."
ros2 topic echo /tb1/odom --once > /dev/null 2>&1 && echo "✅ Odometry OK" || echo "❌ Odometry missing"

echo "2. Checking static transforms (waiting 10s for transforms to be published)..."
sleep 10
timeout 3 ros2 run tf2_ros tf2_echo world tb1/map 2>&1 | grep -q "At time" && echo "✅ Transforms OK" || echo "❌ Transforms missing (run: ros2 run tf2_ros tf2_echo world tb1/map)"

echo "3. Checking map topics..."
MAP_COUNT=$(ros2 topic list | grep -c map)
echo "Found $MAP_COUNT map topics"
[ $MAP_COUNT -ge 3 ] && echo "✅ Maps OK" || echo "❌ Maps missing"

echo "4. Checking merged map..."
ros2 topic echo /map_merged --once > /dev/null 2>&1 && echo "✅ Merged map OK" || echo "❌ Merged map missing"

echo "=== Test Complete ==="
