#!/bin/bash
# Diagnostic script: enumerate USB V4L2 cameras on the system.
# Usage:
#   bash src/yolo_obstacle_detection_ros2/tools/check_camera_usb.sh
#   ros2 run yolo_obstacle_detection_ros2 check_camera_usb.sh
#
# Shows /dev/v4l/by-id, /dev/v4l/by-path, /dev/video*, and if v4l2-ctl
# is available, also shows card name, driver, capabilities, formats, and
# framerates for each device.

set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
BOLD='\033[1m'
RESET='\033[0m'

echo -e "${BOLD}=== USB V4L2 Camera Diagnostic ===${RESET}"
echo ""

# ── Check for v4l-utils ──────────────────────────────────────────────────────────
V4L2CTL=$(command -v v4l2-ctl 2>/dev/null || echo "")
if [ -z "$V4L2CTL" ]; then
    echo -e "${YELLOW}[WARN] v4l2-ctl not found. Install with: sudo apt install v4l-utils${RESET}"
    echo ""
fi

# ── /dev/v4l/by-id ───────────────────────────────────────────────────────────
echo -e "${BOLD}=== /dev/v4l/by-id ===${RESET}"
if [ -d "/dev/v4l/by-id" ]; then
    for f in /dev/v4l/by-id/*; do
        if [ -e "$f" ]; then
            name=$(basename "$f")
            target=$(readlink -f "$f" 2>/dev/null || echo "unknown")
            echo -e "  ${CYAN}$name${RESET} -> ${GREEN}$target${RESET}"
        fi
    done
else
    echo -e "  ${RED}Directory not found${RESET}"
fi
echo ""

# ── /dev/v4l/by-path ──────────────────────────────────────────────────────────
echo -e "${BOLD}=== /dev/v4l/by-path ===${RESET}"
if [ -d "/dev/v4l/by-path" ]; then
    for f in /dev/v4l/by-path/*; do
        if [ -e "$f" ]; then
            name=$(basename "$f")
            target=$(readlink -f "$f" 2>/dev/null || echo "unknown")
            echo -e "  ${CYAN}$name${RESET} -> ${GREEN}$target${RESET}"
        fi
    done
else
    echo -e "  ${RED}Directory not found${RESET}"
fi
echo ""

# ── /dev/video* ───────────────────────────────────────────────────────────────
echo -e "${BOLD}=== /dev/video* ===${RESET}"
found=0
for dev in /dev/video[0-9]*; do
    [ -e "$dev" ] || continue
    found=1
    echo -e "  ${GREEN}$dev${RESET}"

    if [ -n "$V4L2CTL" ]; then
        echo -e "    Card:       ${CYAN}$("$V4L2CTL" -d "$dev" -C 2>/dev/null | head -1 || echo "N/A")${RESET}"
        echo -e "    Driver:     ${CYAN}$("$V4L2CTL" -d "$dev" -D 2>/dev/null | grep "Driver name" | awk '{print $3}' || echo "N/A")${RESET}"
        echo -e "    Bus info:   ${CYAN}$("$V4L2CTL" -d "$dev" -D 2>/dev/null | grep "Bus info" | awk -F: '{print $2}' | xargs || echo "N/A")${RESET}"
    fi
done
if [ "$found" -eq 0 ]; then
    echo -e "  ${RED}No /dev/video* devices found${RESET}"
fi
echo ""

# ── Detailed device info with v4l2-ctl ───────────────────────────────────────
if [ -n "$V4L2CTL" ]; then
    echo -e "${BOLD}=== Device Details (v4l2-ctl) ===${RESET}"
    for dev in /dev/video[0-9]*; do
        [ -e "$dev" ] || continue

        echo -e "\n  ${GREEN}$dev${RESET}"
        echo "  ─────────────────────────────────────────────────────"

        # Driver / device info
        info=$("$V4L2CTL" -d "$dev" -D 2>/dev/null)
        if [ -n "$info" ]; then
            card=$(echo "$info" | grep "Card" | sed 's/.*: *//')
            driver=$(echo "$info" | grep "Driver name" | sed 's/.*: *//')
            bus=$(echo "$info" | grep "Bus info" | sed 's/.*: *//')
            chip=$(echo "$info" | grep "Chip ID" | sed 's/.*: *//')
            echo -e "  Card:     ${CYAN}$card${RESET}"
            echo -e "  Driver:   ${CYAN}$driver${RESET}"
            echo -e "  Bus:      ${CYAN}$bus${RESET}"
            if [ -n "$chip" ]; then
                echo -e "  Chip ID:  ${CYAN}$chip${RESET}"
            fi
        fi

        # Capabilities
        echo -e "  ${BOLD}Capabilities:${RESET}"
        caps=$("$V4L2CTL" -d "$dev" -l 2>/dev/null | head -1)
        if [[ "$caps" == *"capture"* ]]; then
            echo -e "    ${GREEN}[CAPTURE]${RESET} Video capture supported"
        fi
        if [[ "$caps" == *"streaming"* ]]; then
            echo -e "    ${GREEN}[STREAM]${RESET} Streaming/IO modes supported"
        fi

        # Supported formats
        echo -e "  ${BOLD}Pixel Formats:${RESET}"
        formats=$("$V4L2CTL" -d "$dev" --list-formats 2>/dev/null | grep "\[" || true)
        if [ -n "$formats" ]; then
            echo "$formats" | while IFS= read -r line; do
                echo -e "    ${CYAN}$line${RESET}"
            done
        else
            echo -e "    ${YELLOW}(none listed)${RESET}"
        fi

        # Frame sizes for MJPG (preferred)
        echo -e "  ${BOLD}Frame Sizes (MJPG):${RESET}"
        sizes=$("$V4L2CTL" -d "$dev" --list-formats-ext 2>/dev/null | grep -A20 "MJPG" | grep "Size" || true)
        if [ -n "$sizes" ]; then
            echo "$sizes" | while IFS= read -r line; do
                echo -e "    ${GREEN}$line${RESET}"
            done
        else
            echo -e "    ${YELLOW}(not listed)${RESET}"
        fi

        # Frame intervals for 1280x720
        echo -e "  ${BOLD}Frame Intervals @ 1280x720:${RESET}"
        intervals=$("$V4L2CTL" -d "$dev" --list-formats-ext 2>/dev/null | grep -A30 "1280x720" | grep "Interval" | head -5 || true)
        if [ -n "$intervals" ]; then
            echo "$intervals" | while IFS= read -r line; do
                echo -e "    ${GREEN}$line${RESET}"
            done
        else
            echo -e "    ${YELLOW}(not listed)${RESET}"
        fi
    done
fi

echo ""
echo -e "${BOLD}=== Done ===${RESET}"
echo ""
echo "Tip: If the camera appears at a different /dev/videoX after reboot,"
echo "      use enable_camera:=true camera_device:=auto — the driver will"
echo "      automatically find the device via /dev/v4l/by-id stable alias."
