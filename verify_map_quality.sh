#!/bin/bash
echo "=== Map Quality Verification ==="
echo ""

echo "1. Checking transform integration with map_merge..."
echo "   Verifying world -> tb1/map transform:"
timeout 3 ros2 run tf2_ros tf2_echo world tb1/map 2>&1 | head -10
echo ""

echo "2. Checking map resolution:"
ros2 param get /tb1/slam_toolbox resolution 2>/dev/null || echo "   Could not get resolution parameter"
echo ""

echo "3. Checking map merge parameters:"
ros2 param list /map_merge 2>/dev/null | grep -E "(world_frame|merging_rate|confidence)" || echo "   Could not get map_merge parameters"
echo ""

echo "4. Checking map topics and their sizes:"
for topic in /tb1/map /tb2/map /map_merged; do
    echo "   Checking $topic..."
    ros2 topic echo $topic --once 2>/dev/null | grep -E "(width|height|resolution)" | head -3 || echo "   Topic not publishing"
done
echo ""

echo "5. Checking TF tree for transform chain:"
ros2 run tf2_tools view_frames 2>&1 | tail -3
echo "   Check frames.pdf for complete transform tree"
echo ""

echo "=== Verification Complete ==="
