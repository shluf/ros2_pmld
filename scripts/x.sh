#!/bin/bash

WORKSPACE=$(pwd)
INIT_SCRIPT="$WORKSPACE/scripts/init.sh"

function show_menu() {
    clear
    echo "================================================"
    echo "          ROS2 Tello Control Menu               "
    echo "================================================"
    echo ""
    echo "  [SETUP]"
    echo "  1) Build workspace"
    echo ""
    echo "  [SIMULATION]"
    echo "  2) Gazebo simulation only"
    echo "  3) Gesture Control (Gazebo + Debug)"
    echo "  4) Gesture Control (Gazebo + Production)"
    echo ""
    echo "  [GUI CONTROL]"
    echo "  5) Tello Control GUI"
    echo ""
    echo "  [REAL DRONE]"
    echo "  6) Connect to REAL drone"
    echo "  7) Gesture Control (Real Drone)"
    echo ""
    echo "  0) Exit"
    echo "================================================"
    read -p "Pilih opsi [0-7]: " choice
}

function build_workspace() {
    echo "================================================"
    echo "            Building Workspace                  "
    echo "================================================"
    echo ""
    cd "$WORKSPACE"
    colcon build --symlink-install
    echo ""
    echo "Build complete!"
}

function run_simulation() {
    echo "================================================"
    echo "        Launching Gazebo Simulation             "
    echo "================================================"
    echo ""
    source $INIT_SCRIPT
    source $WORKSPACE/scripts/kill_gazebo.sh
    ros2 launch tello_activation sim_drone.launch.py
}

function run_gesture_control_gazebo_debug() {
    echo "================================================"
    echo "    Gesture Control - Gazebo Debug Mode         "
    echo "================================================"
    echo ""
    echo "Mode ini untuk testing dan debugging:"
    echo "  * Gazebo simulation"
    echo "  * Debug visualization window"
    echo "  * Extended logging"
    echo "  * Safety features DISABLED"
    echo "  * Gesture hold time: 0.5s (fast response)"
    echo ""
    read -p "Press Enter to launch..."
    
    source $INIT_SCRIPT
    source $WORKSPACE/scripts/kill_gazebo.sh
    ros2 launch gesture_control debug_gazebo_launch.py
}

function run_gesture_control_gazebo_prod() {
    echo "================================================"
    echo "   Gesture Control - Gazebo Production Mode     "
    echo "================================================"
    echo ""
    echo "Mode ini untuk testing production settings:"
    echo "  * Gazebo simulation"
    echo "  * Safety features ENABLED"
    echo "  * Gesture hold time: 1.0s (safer)"
    echo "  * No debug visualization"
    echo ""
    read -p "Press Enter to launch..."
    
    source $INIT_SCRIPT
    source $WORKSPACE/scripts/kill_gazebo.sh
    ros2 launch gesture_control gesture_control_launch.py \
        namespace:=drone1 \
        use_drone_camera:=true \
        debug_mode:=false \
        enable_safety:=true \
        simulation:=true
}

function run_gesture_control_real() {
    echo "================================================"
    echo "      Gesture Control - Real Drone              "
    echo "================================================"
    echo ""
    echo "WARNING - SAFETY CHECKLIST:"
    echo "  [ ] Drone sudah terhubung ke WiFi"
    echo "  [ ] Area terbang aman dan luas (min 2x2 meter)"
    echo "  [ ] Tidak ada halangan di sekitar"
    echo "  [ ] Siap untuk emergency landing"
    echo ""
    echo "Pilih mode:"
    echo "  1) Production (Safety ON, Hold 1.0s)"
    echo "  2) Debug (Safety OFF, Hold 0.5s, Visualization)"
    echo "  0) Cancel"
    echo ""
    read -p "Mode [0-2]: " mode_choice
    
    case $mode_choice in
        1)
            echo ""
            echo "[INFO] Launching Production Mode..."
            echo "       - Safety features: ENABLED"
            echo "       - Gesture hold time: 1.0s"
            echo ""
            source $INIT_SCRIPT
            ros2 launch gesture_control gesture_control_launch.py \
                namespace:=drone1 \
                use_drone_camera:=true \
                debug_mode:=false \
                enable_safety:=true \
                simulation:=false
            ;;
        2)
            echo ""
            echo "[INFO] Launching Debug Mode..."
            echo "       - Safety features: DISABLED"
            echo "       - Gesture hold time: 0.5s"
            echo "       - Debug visualization: ON"
            echo ""
            source $INIT_SCRIPT
            ros2 launch gesture_control gesture_control_launch.py \
                namespace:=drone1 \
                use_drone_camera:=true \
                debug_mode:=true \
                enable_safety:=false \
                simulation:=false
            ;;
        *)
            echo "Cancelled."
            return
            ;;
    esac
}

function run_tello_gui() {
    echo "================================================"
    echo "            Tello Control GUI                   "
    echo "================================================"
    echo ""
    echo "Pilih mode operasi:"
    echo ""
    echo "  1) Simulation (Gazebo + GUI + Gesture)"
    echo "  2) Real Drone (Driver + GUI + Gesture)"
    echo "  3) GUI Only (no drone, no gesture)"
    echo "  0) Cancel"
    echo ""
    read -p "Mode [0-3]: " gui_mode
    
    source $INIT_SCRIPT
    
    case $gui_mode in
        1)
            echo ""
            echo "================================================"
            echo "       Launching: Simulation Mode               "
            echo "================================================"
            echo ""
            echo "Components:"
            echo "  * Gazebo simulation"
            echo "  * Tello Control GUI"
            echo "  * Gesture Control (optional)"
            echo ""
            read -p "Enable Gesture Control? [y/n]: " gesture
            
            if [[ "$gesture" == "y" || "$gesture" == "Y" ]]; then
                echo "[INFO] Launching with Gesture Control..."
                ros2 launch tello_control_gui tello_gui_launch.py \
                    with_driver:=false \
                    with_gesture:=true \
                    simulation:=true
            else
                echo "[INFO] Launching without Gesture Control..."
                ros2 launch tello_control_gui tello_gui_launch.py \
                    with_driver:=false \
                    with_gesture:=false \
                    simulation:=true
            fi
            ;;
        2)
            echo ""
            echo "================================================"
            echo "        Launching: Real Drone Mode              "
            echo "================================================"
            echo ""
            echo "WARNING - SAFETY CHECK:"
            echo "  [ ] Terhubung ke WiFi TELLO-XXXXXX"
            echo "  [ ] Drone sudah menyala"
            echo "  [ ] Area aman untuk terbang"
            echo ""
            read -p "Continue? [y/n]: " confirm
            
            if [[ "$confirm" == "y" || "$confirm" == "Y" ]]; then
                echo ""
                read -p "Enable Gesture Control? [y/n]: " gesture
                
                if [[ "$gesture" == "y" || "$gesture" == "Y" ]]; then
                    echo "[INFO] Launching with Driver + GUI + Gesture..."
                    ros2 launch tello_control_gui tello_gui_launch.py \
                        with_driver:=true \
                        with_gesture:=true \
                        simulation:=false
                else
                    echo "[INFO] Launching with Driver + GUI only..."
                    ros2 launch tello_control_gui tello_gui_launch.py \
                        with_driver:=true \
                        with_gesture:=false \
                        simulation:=false
                fi
            else
                echo "Cancelled."
            fi
            ;;
        3)
            echo ""
            echo "[INFO] Launching GUI only (no drone, no gesture)..."
            ros2 launch tello_control_gui tello_gui_launch.py \
                with_driver:=false \
                with_gesture:=false \
                simulation:=false
            ;;
        *)
            echo "Cancelled."
            ;;
    esac
}

function connect_tello() {
    echo "================================================"
    echo "         Connecting to Real Drone               "
    echo "================================================"
    echo ""
    source $INIT_SCRIPT
    source "$WORKSPACE/scripts/connect_tello.sh"
}

if [ $# -gt 0 ]; then
    case "$1" in
        build)
            build_workspace
            return 0 2>/dev/null || exit 0
            ;;
        sim|gazebo)
            source "$INIT_SCRIPT"
            source "$WORKSPACE/scripts/kill_gazebo.sh"
            ros2 launch tello_activation sim_drone.launch.py
            return 0 2>/dev/null || exit 0
            ;;
        gesture-debug|gd)
            source "$INIT_SCRIPT"
            source "$WORKSPACE/scripts/kill_gazebo.sh"
            ros2 launch gesture_control debug_gazebo_launch.py
            return 0 2>/dev/null || exit 0
            ;;
        gesture-prod|gp)
            source "$INIT_SCRIPT"
            source "$WORKSPACE/scripts/kill_gazebo.sh"
            ros2 launch gesture_control gesture_control_launch.py \
                namespace:=drone1 \
                use_drone_camera:=true \
                debug_mode:=false \
                enable_safety:=true \
                simulation:=true
            return 0 2>/dev/null || exit 0
            ;;
        gesture-real|gr)
            source "$INIT_SCRIPT"
            echo "[INFO] Launching Gesture Control for Real Drone (Production Mode)..."
            ros2 launch gesture_control gesture_control_launch.py \
                namespace:=drone1 \
                use_drone_camera:=true \
                debug_mode:=false \
                enable_safety:=true \
                simulation:=false
            return 0 2>/dev/null || exit 0
            ;;
        gui)
            source "$INIT_SCRIPT"
            ros2 launch tello_control_gui tello_gui_launch.py
            return 0 2>/dev/null || exit 0
            ;;
        real|connect)
            source "$INIT_SCRIPT"
            echo "[INFO] Launching real drone connection..."
            source "$WORKSPACE/scripts/connect_tello.sh"
            return 0 2>/dev/null || exit 0
            ;;
        help|-h|--help)
            echo "╔════════════════════════════════════════════════╗"
            echo "║       ROS2 Tello - Command Line Usage          ║"
            echo "╚════════════════════════════════════════════════╝"
            echo ""
            echo "Usage: $0 [COMMAND]"
            echo ""
            echo "COMMANDS:"
            echo ""
            echo "  Setup:"
            echo "    build              Build workspace"
            echo ""
            echo "  Simulation:"
            echo "    sim, gazebo        Run Gazebo simulation only"
            echo "    gesture-debug, gd  Gesture Control (Gazebo Debug)"
            echo "    gesture-prod, gp   Gesture Control (Gazebo Production)"
            echo ""
            echo "  GUI:"
            echo "    gui                Launch Tello Control GUI"
            echo ""
            echo "  Real Drone:"
            echo "    real, connect      Connect to real Tello drone"
            echo "    gesture-real, gr   Gesture Control (Real Drone)"
            echo ""
            echo "  Help:"
            echo "    help, -h, --help   Show this help message"
            echo ""
            echo "EXAMPLES:"
            echo "  $0 build            # Build workspace"
            echo "  $0 gd               # Quick gesture debug mode"
            echo "  $0 gui              # Launch GUI"
            echo "  $0 real             # Connect to real drone"
            echo ""
            return 0 2>/dev/null || exit 0
            ;;
        *)
            echo "❌ [ERROR] Unknown command: $1"
            echo ""
            echo "Use '$0 help' to see available commands."
            echo ""
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
            run_gesture_control_gazebo_debug
            ;;
        4)
            run_gesture_control_gazebo_prod
            ;;
        5)
            run_tello_gui
            ;;
        6)
            connect_tello
            ;;
        7)
            run_gesture_control_real
            ;;
        0|q|Q)
            echo ""
            echo "Keluar dari menu. Bye!"
            break  
            ;;
        *)
            echo ""
            echo "Pilihan tidak valid!"
            ;;
    esac
    echo ""
    echo "------------------------------------------------"
    read -p "Tekan [Enter] untuk kembali ke menu..."
done