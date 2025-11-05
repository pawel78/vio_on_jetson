#!/bin/bash
# Record stereo camera and IMU data to rosbag

set -e

# Default values
OUTPUT_DIR="bags"
BAG_NAME="stereo_vo_$(date +%Y%m%d_%H%M%S)"
DURATION=0  # 0 = infinite
COMPRESSION="zstd"

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -o|--output)
            OUTPUT_DIR="$2"
            shift 2
            ;;
        -n|--name)
            BAG_NAME="$2"
            shift 2
            ;;
        -d|--duration)
            DURATION="$2"
            shift 2
            ;;
        -c|--compression)
            COMPRESSION="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -o, --output DIR       Output directory (default: bags)"
            echo "  -n, --name NAME        Bag name (default: stereo_vo_TIMESTAMP)"
            echo "  -d, --duration SEC     Recording duration in seconds (default: infinite)"
            echo "  -c, --compression TYPE Compression type: none, zstd, lz4 (default: zstd)"
            echo "  -h, --help             Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Create output directory
mkdir -p "$OUTPUT_DIR"

BAG_PATH="$OUTPUT_DIR/$BAG_NAME"

echo "========================================="
echo "Recording ROS 2 Bag"
echo "========================================="
echo "Output: $BAG_PATH"
echo "Duration: $([ $DURATION -eq 0 ] && echo 'infinite' || echo ${DURATION}s)"
echo "Compression: $COMPRESSION"
echo "========================================="
echo ""

# Topics to record
TOPICS=(
    "/stereo/left/image_raw"
    "/stereo/right/image_raw"
    "/stereo/left/camera_info"
    "/stereo/right/camera_info"
    "/imu/data"
    "/vo/odom"
    "/tf"
    "/tf_static"
)

# Build ros2 bag record command
CMD="ros2 bag record"

# Add topics
for topic in "${TOPICS[@]}"; do
    CMD="$CMD $topic"
done

# Add output
CMD="$CMD -o $BAG_PATH"

# Add compression
if [ "$COMPRESSION" != "none" ]; then
    CMD="$CMD --compression-mode file --compression-format $COMPRESSION"
fi

# Add duration if specified
if [ $DURATION -gt 0 ]; then
    CMD="$CMD --max-cache-size 0 --duration $DURATION"
fi

echo "Recording topics:"
for topic in "${TOPICS[@]}"; do
    echo "  - $topic"
done
echo ""
echo "Press Ctrl+C to stop recording"
echo ""

# Execute
eval $CMD

echo ""
echo "========================================="
echo "Recording complete!"
echo "Bag saved to: $BAG_PATH"
echo ""
echo "Inspect bag:"
echo "  ros2 bag info $BAG_PATH"
echo ""
echo "Play bag:"
echo "  ros2 bag play $BAG_PATH"
echo "========================================="
