#!/bin/bash

WORKSPACE=~/ros2_pmld
INIT_SCRIPT="$WORKSPACE/scripts/init.sh"

function show_menu() {
    clear
    echo "=============================================="
    echo "                ROS2 Tello Menu               "
    echo "=============================================="
    echo "1) Build workspace"
    echo "2) Run Gazebo simulation"
    echo "3) Clean & relaunch Gazebo sim"
    echo "4) Run keyboard controller"
    echo "0) Exit"
    echo "=============================================="
    read -p "Pilih opsi [0-4]: " choice
}

function build_workspace() {
    echo "[INFO] Building workspace..."
    cd "$WORKSPACE"
    colcon build --symlink-install
    echo "Build complete."
}

function run_simulation() {
    echo "[INFO] Launching Gazebo simulation..."
    gnome-terminal -- bash -c "source $INIT_SCRIPT; ros2 launch tello_gazebo simple_launch.py; exec bash"
}

function run_keyboard() {
    echo "[INFO] Running keyboard controller..."
    source $INIT_SCRIPT
    ros2 run tello_keyboard keyboard_controller --ros-args -p namespace:=drone1
}

function relaunch_tello_sim() {
    local default_port=11345
    echo "[INFO] This will kill any running Gazebo and relaunch the Tello sim on a custom port."
    read -p "Masukkan port Gazebo [default: ${default_port}]: " port
    port=${port:-$default_port}

    echo "[INFO] Killing Gazebo processes and freeing port ${port}..."
    bash "$WORKSPACE/scripts/kill_gazebo.sh" "$port"

    echo "[INFO] Launching Tello Gazebo on port ${port}..."
    local model_path="$WORKSPACE/install/tello_gazebo/share/tello_gazebo/models"
    gnome-terminal -- bash -c "\
        source $INIT_SCRIPT; \
        export GAZEBO_MODEL_PATH=${model_path}; \
        source /usr/share/gazebo/setup.sh; \
        export GAZEBO_MASTER_URI=http://127.0.0.1:${port}; \
        ros2 launch tello_gazebo simple_launch.py; \
        exec bash"
}

if [ $# -gt 0 ]; then
    case "$1" in
        build)
            build_workspace
            return 0 2>/dev/null || exit 0
            ;;
        run-sim|sim)
            run_simulation
            return 0 2>/dev/null || exit 0
            ;;
        keyboard|kb)
            source "$INIT_SCRIPT"
            ros2 run tello_keyboard keyboard_controller --ros-args -p namespace:=drone1
            return 0 2>/dev/null || exit 0
            ;;
        relaunch-sim|rsim)
            port=${2:-11345}
            echo "[INFO] Using port ${port}"
            bash "$WORKSPACE/scripts/kill_gazebo.sh" "$port"
            model_path="$WORKSPACE/install/tello_gazebo/share/tello_gazebo/models"
            echo "[INFO] Launching Tello Gazebo (no new terminal) on port ${port}..."
            source "$INIT_SCRIPT"
            export GAZEBO_MODEL_PATH="${model_path}"
            source /usr/share/gazebo/setup.sh
            export GAZEBO_MASTER_URI="http://127.0.0.1:${port}"
            ros2 launch tello_gazebo simple_launch.py
            return 0 2>/dev/null || exit 0
            ;;
        help|-h|--help)
            echo "Usage: $0 [build|run-sim|keyboard|sim-keyboard|help]"
            echo ""
            echo "  build       : Build workspace"
            echo "  run-sim     : Jalankan simulasi Gazebo"
            echo "  keyboard    : Jalankan keyboard controller"
            echo "  help        : Tampilkan bantuan"
            echo ""
            echo "Advanced:"
            echo "  relaunch-sim [PORT] : Kill Gazebo & launch Tello sim on custom port (default 11345)"
            return 0 2>/dev/null || exit 0
            ;;
        *)
            echo "[ERROR] Argumen tidak dikenal: $1"
            echo "Gunakan '$0 help' untuk melihat opsi yang tersedia."
            return 1 2>/dev/null || exit 1
            ;;
    esac
fi

while true; do
    show_menu
    case $choice in
        1)
            build_workspace
            ;;
        2)
            run_simulation
            ;;
        3)
            relaunch_tello_sim
            ;;
        4)
            run_keyboard
            ;;
        0|q|Q)
            echo "Keluar dari menu. Bye!"
            break  
            ;;
        *)
            echo "Pilihan tidak valid!"
            ;;
    esac
    echo ""
    read -p "Tekan [Enter] untuk kembali ke menu..."
done