#!/bin/bash

WORKSPACE=$(pwd)
INIT_SCRIPT="$WORKSPACE/scripts/init.sh"


echo "╔═══════════════════════════════════════════════════════════╗"
echo "║                 KONEKSI KE DRONE TELLO                    ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "LANGKAH-LANGKAH:"
echo ""
echo "[1]  Nyalakan drone Tello"
echo "    - Tekan tombol power pada drone"
echo "    - Tunggu LED berkedip (drone siap)"
echo ""
echo "[2]  Hubungkan ke WiFi drone"
echo "    - Buka WiFi settings di laptop"
echo "    - Cari jaringan: TELLO-XXXXXX"
echo "    - Connect ke jaringan tersebut"
echo "    - Password: (biasanya tidak ada password)"
echo ""
echo "[3]  Verifikasi koneksi"
echo "    - IP drone: 192.168.10.1"
echo "    - Test ping..."
echo ""

# Check WiFi connection
if ping -c 1 -W 2 192.168.10.1 &> /dev/null; then
    echo "[SUCCESS] Drone terhubung! (192.168.10.1)"
    CONNECTED=true
else
    echo "[ERROR] Drone tidak terdeteksi"
    echo ""
    echo "TROUBLESHOOTING:"
    echo "  • Pastikan laptop terhubung ke WiFi TELLO-XXXXXX"
    echo "  • Pastikan drone sudah menyala (LED berkedip)"
    echo "  • Coba disconnect dan connect ulang ke WiFi"
    echo "  • Restart drone jika perlu"
    echo ""
    read -p "Lanjutkan anyway? (y/n): " answer
    if [[ ! "$answer" =~ ^[Yy]$ ]]; then
        echo "Dibatalkan."
        # exit 1
    fi
    CONNECTED=false
fi

echo ""
echo "╔═══════════════════════════════════════════════════════════╗"
echo "║                    PILIH MODE                             ║"
echo "╚═══════════════════════════════════════════════════════════╝"
echo ""
echo "1) Driver only"
echo "2) Driver + Keyboard controller"
echo "3) Driver + Joystick controller"
echo "0) Cancel"
echo ""
read -p "Pilih mode [0-3]: " mode

case $mode in
    1)
        echo ""
        echo "   Launching tello_driver only..."
        echo "   Monitoring topics:"
        echo "   - ros2 topic echo /flight_data"
        echo "   - ros2 topic echo /tello_response"
        echo ""
        source "$INIT_SCRIPT"
        ros2 run tello_driver tello_driver_main
        ;;
    2)
        echo ""
        echo "   Launching tello_driver + keyboard controller..."
        echo ""
        echo "   SAFETY TIPS:"
        echo "   • Pastikan area terbang aman dan luas"
        echo "   • Siapkan tombol H untuk emergency stop"
        echo "   • Mulai dengan takeoff (T) terlebih dahulu"
        echo "   • Test gerakan pelan-pelan dulu"
        echo ""
        read -p "Press Enter to continue..."
        
        source "$INIT_SCRIPT"
        ros2 launch tello_activation real_drone.launch.py
        ;;
    3)
        echo ""
        echo "   Launching tello_driver + joystick..."
        echo "   Make sure joystick is connected!"
        echo ""
        source "$INIT_SCRIPT"
        ros2 launch tello_driver teleop_launch.py
        ;;
    0|*)
        echo "Cancelled."
        # exit 0
        ;;
esac
