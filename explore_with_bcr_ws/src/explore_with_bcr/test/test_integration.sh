#!/bin/bash
# Integration test for explore_with_bcr
# Validates that all components are properly configured

echo "=========================================="
echo "explore_with_bcr Integration Tests"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
NC='\033[0m' # No Color

PASS_COUNT=0
FAIL_COUNT=0

# Test function
run_test() {
    local test_name="$1"
    local test_cmd="$2"

    echo -n "Testing: $test_name ... "
    if eval "$test_cmd" > /dev/null 2>&1; then
        echo -e "${GREEN}PASS${NC}"
        ((PASS_COUNT++))
    else
        echo -e "${RED}FAIL${NC}"
        ((FAIL_COUNT++))
    fi
}

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="$(dirname "$PKG_DIR")"

echo "1. Package Structure Tests"
echo "--------------------------"
run_test "CMakeLists.txt exists" "test -f $PKG_DIR/CMakeLists.txt"
run_test "package.xml exists" "test -f $PKG_DIR/package.xml"
run_test "README.md exists" "test -f $PKG_DIR/README.md"
run_test "explore_with_mapping.launch.py exists" "test -f $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "explore_only.launch.py exists" "test -f $PKG_DIR/launch/explore_only.launch.py"
run_test "tdle_params.yaml exists" "test -f $PKG_DIR/config/tdle_params.yaml"
run_test "start_exploration.sh exists" "test -f $PKG_DIR/start_exploration.sh"

echo ""
echo "2. Launch File Syntax Tests"
echo "--------------------------"
run_test "explore_with_mapping.launch.py compiles" "python3 -m py_compile $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "explore_only.launch.py compiles" "python3 -m py_compile $PKG_DIR/launch/explore_only.launch.py"

echo ""
echo "3. Topic Configuration Tests"
echo "--------------------------"
run_test "odom_topic is /bcr_bot/odom in config" "grep -q '/bcr_bot/odom' $PKG_DIR/config/tdle_params.yaml"
run_test "map_topic is /map in config" "grep -q '/map' $PKG_DIR/config/tdle_params.yaml"
run_test "costmap_topic is /global_costmap/costmap" "grep -q '/global_costmap/costmap' $PKG_DIR/config/tdle_params.yaml"

echo ""
echo "4. Launch File Content Tests"
echo "--------------------------"
run_test "fpoint_detector in launch file" "grep -q 'fpoint_detector' $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "fpoint_filter in launch file" "grep -q 'fpoint_filter' $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "subregion_arrangement in launch file" "grep -q 'subregion_arrangement' $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "fpoint_selection in launch file" "grep -q 'fpoint_selection' $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "SLAM Toolbox in launch file" "grep -q 'slam_toolbox' $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "Nav2 in launch file" "grep -q 'nav2' $PKG_DIR/launch/explore_with_mapping.launch.py"
run_test "Gazebo in launch file" "grep -q 'gz_sim' $PKG_DIR/launch/explore_with_mapping.launch.py"

echo ""
echo "5. Dependency Tests"
echo "--------------------------"
run_test "Depends on bcr_bot" "grep -q 'bcr_bot' $PKG_DIR/package.xml"
run_test "Depends on tdle" "grep -q 'tdle' $PKG_DIR/package.xml"
run_test "Depends on slam_toolbox" "grep -q 'slam_toolbox' $PKG_DIR/package.xml"
run_test "Depends on nav2_bringup" "grep -q 'nav2_bringup' $PKG_DIR/package.xml"

echo ""
echo "=========================================="
echo "Test Results: $PASS_COUNT passed, $FAIL_COUNT failed"
echo "=========================================="

if [ $FAIL_COUNT -eq 0 ]; then
    echo -e "${GREEN}All tests passed!${NC}"
    exit 0
else
    echo -e "${RED}Some tests failed!${NC}"
    exit 1
fi
