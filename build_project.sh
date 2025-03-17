#!/bin/bash

# 🔹 Skript na automatický build projektu

# Vyčistenie premennných pred buildom
unset AMENT_PREFIX_PATH
unset CMAKE_PREFIX_PATH

# Zdrojovanie ROS2 Jazzy
source /opt/ros/jazzy/setup.bash

# Zdrojovanie Gazebo Harmony 8
if [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
fi

# Vytvorenie priečinka pre logy
BUILD_LOG_DIR="$PWD/moje_build_logs"
mkdir -p "$BUILD_LOG_DIR"

TIMESTAMP=$(date +'%d_%m_%Y_%H:%M:%S')
FULL_BUILD_DIR="$BUILD_LOG_DIR/$TIMESTAMP"
mkdir -p "$FULL_BUILD_DIR"

# Definícia logu
BUILD_LOG="$FULL_BUILD_DIR/build_log.txt"

# Vyčistenie starých logov pred buildom
if [ -d "./log" ]; then
    rm -rf ./log/*
fi

# rm -rf build/ install/ log/* || true

# Spustíme build
colcon build --symlink-install 2>&1 | tee "$BUILD_LOG"
BUILD_EXIT_CODE=${PIPESTATUS[0]}

# Ak build padne skript sa vypne a vypíše chybu
if [ $BUILD_EXIT_CODE -ne 0 ]; then
  printf "❌ CHYBA!!! Build projektu zlyhal, pozrite si priečinok logu: %s\n" "$FULL_BUILD_DIR"
  exit 1
fi

# Vymazanie starých build súborov (až po kopírovaní logov)
if [ -d "./log" ]; then
    cp -r ./log/* "$FULL_BUILD_DIR/"
    printf "✅ Logy boli skopírované do: %s\n" "$FULL_BUILD_DIR/"
else
    printf "⚠️ VAROVANIE: Priečinok /log neexistuje, nebolo čo skopírovať.\n"
fi

# Zdrojovanie po builde
source install/setup.bash

# Oprava environmentálnych premenných pre ROS2 a môj projekt
export AMENT_PREFIX_PATH=/opt/ros/jazzy:$AMENT_PREFIX_PATH
export CMAKE_PREFIX_PATH=$HOME/ros2_ws/install:$CMAKE_PREFIX_PATH

# Oprava premenných pre Gazebo
export GAZEBO_MODEL_PATH=$HOME/ros2_ws/src/jetbot_gazebo/models:$GAZEBO_MODEL_PATH
export GAZEBO_RESOURCE_PATH=$HOME/ros2_ws/src/jetbot_gazebo/worlds:$GAZEBO_RESOURCE_PATH
export GAZEBO_PLUGIN_PATH=$HOME/ros2_ws/build/jetbot_gazebo

printf "✅ Build hotový, prostredie je nastavené\n"
printf "✅ Projekt spustený!\n"
