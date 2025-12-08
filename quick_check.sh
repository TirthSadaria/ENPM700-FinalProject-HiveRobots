#!/bin/bash
# Quick verification script after fixes

echo "=== Quick System Check ==="
echo ""

echo "1. Checking if system is running..."
if pgrep -f "hive_slam.launch.py" > /dev/null; then
    echo "   ✅ SLAM system is running"
    
    echo ""
    echo "2. Checking resolution parameter..."
    RESOLUTION=$(ros2 param get /tb1/slam_toolbox resolution 2>/dev/null | grep -oP 'Double value is: \K[0-9.]+' || echo "not_found")
    if [ "$RESOLUTION" = "0.03" ]; then
        echo "   ✅ Resolution is 0.03m (3cm) - CORRECT"
    elif [ "$RESOLUTION" = "not_found" ]; then
        echo "   ⚠️  Could not read resolution (node might not be ready)"
    else
        echo "   ❌ Resolution is $RESOLUTION (expected 0.03)"
    fi
    
    echo ""
    echo "3. Checking transform chain..."
    sleep 2
    if timeout 5 ros2 run tf2_ros tf2_echo world map 2>&1 | grep -q "Translation:"; then
        echo "   ✅ Transform chain (world -> map) is working"
    else
        echo "   ❌ Transform chain broken (run: ros2 run tf2_ros tf2_echo world map)"
    fi
    
    echo ""
    echo "4. Checking map topics..."
    MAP_COUNT=$(ros2 topic list | grep -c "/map")
    echo "   Found $MAP_COUNT map-related topics"
    if [ $MAP_COUNT -ge 3 ]; then
        echo "   ✅ Maps are publishing"
    else
        echo "   ⚠️  Expected at least 3 map topics"
    fi
    
else
    echo "   ⚠️  SLAM system is not running"
    echo "   Start with: ros2 launch hive_control hive_slam.launch.py num_robots:=2"
fi

echo ""
echo "=== Check Complete ==="

