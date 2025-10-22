#!/bin/bash

WORKSPACE=~/ros2_pmld
PKG_NAME="follower"
INIT_SCRIPT="$WORKSPACE/scripts/init.sh"

function show_menu() {
    clear
    echo "=============================================="
    echo "        ROS2 Humble Line Follower             "
    echo "=============================================="
    echo "1) Build workspace"
    echo "2) Run Gazebo simulation"
    echo "3) Run follower node"
    echo "4) Start follower service"
    echo "5) Exit"
    echo "=============================================="
    read -p "Pilih opsi [1-5]: " choice
}

function build_workspace() {
    source $INIT_SCRIPT

    echo "[INFO] Building workspace..."
    cd "$WORKSPACE"
    colcon build --symlink-install
    echo "[SUCCESS] Build complete."
}

function run_simulation() {
    source $INIT_SCRIPT
    echo "[INFO] Launching Gazebo simulation..."
    gnome-terminal -- bash -c "source $INIT_SCRIPT; ros2 launch $PKG_NAME new_track.launch.py; exec bash"
}

function run_node() {
    source $INIT_SCRIPT
    echo "[INFO] Running follower node..."
    gnome-terminal -- bash -c "source $INIT_SCRIPT; ros2 run $PKG_NAME follower_node; exec bash"
}

function start_service() {
    source $INIT_SCRIPT
    echo "[INFO] Calling start service..."
    ros2 service call /start_follower std_srvs/srv/Empty
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
        run-node|node)
            run_node
            return 0 2>/dev/null || exit 0
            ;;
        start|service)
            start_service
            return 0 2>/dev/null || exit 0
            ;;
        help|-h|--help)
            echo "Usage: $0 [build|run-sim|run-node|start|help]"
            echo ""
            echo "  build       : Build workspace"
            echo "  run-sim     : Jalankan simulasi Gazebo"
            echo "  run-node    : Jalankan node follower"
            echo "  start       : Jalankan service /start_follower"
            echo "  help        : Tampilkan bantuan"
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
            run_node
            ;;
        4)
            start_service
            ;;
        5)
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