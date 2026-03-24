// =============================================================================
// test_joystick — Gamepad connectivity and axis mapping verification
//
// Purpose:
//   Verify that the gamepad device is readable and that axis indices match
//   the expected MPC velocity command mapping before running the full controller.
//
// Usage:
//   ./build/test_joystick [config/robot_params.yaml]
//
// No hardware (motors/IMU) required. No sudo needed.
//
// Expected axis mapping (from CommandComponent / main.cpp):
//   Axis 1 (left  stick up/down)   ->  vx   = -axis1 * 0.5  [m/s]
//   Axis 3 (right stick left/right) -> yaw  = -axis3 * 0.5  [rad/s]
// =============================================================================

#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <thread>
#include <cstdio>
#include <cstring>

#include "observations.hpp"
#include "common/robot_config.hpp"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// Draw a simple ASCII bar: value in [-1,1], width = 2*half_width+1
static std::string bar(float val, int half_width = 10) {
    int center = half_width;
    int pos    = center + static_cast<int>(val * half_width);
    if (pos < 0) pos = 0;
    if (pos > 2 * half_width) pos = 2 * half_width;

    std::string s(2 * half_width + 1, '-');
    s[center] = '|';   // center marker
    s[pos]    = '#';   // current value
    return "[" + s + "]";
}

int main(int argc, char* argv[]) {
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::string config_path = (argc > 1) ? argv[1] : "config/robot_params.yaml";
    RobotConfig cfg = RobotConfig::from_yaml(config_path);

    std::cout << "=== Joystick Test ===\n";
    std::cout << "Device: " << cfg.gamepad_dev << "\n";
    std::cout << "Ctrl+C to exit\n\n";

    Gamepad gamepad(cfg.gamepad_dev.c_str());

    if (!gamepad.IsConnected()) {
        std::cerr << "[ERROR] Cannot open " << cfg.gamepad_dev << "\n"
                  << "  Check: ls /dev/input/js*\n"
                  << "  Try:   sudo modprobe joydev\n";
        return 1;
    }

    std::cout << "[OK] Gamepad connected.\n\n";

    // Axes of interest (extend if your controller has more)
    // Axis names match the Linux joystick standard layout for most gamepads.
    const char* AXIS_NAMES[] = {
        "0 L-stick LR ",   // axis 0
        "1 L-stick UD ",   // axis 1  -> vx
        "2 L-trigger  ",   // axis 2
        "3 R-stick LR ",   // axis 3  -> yaw
        "4 R-stick UD ",   // axis 4
        "5 R-trigger  ",   // axis 5
    };
    const int N_AXES = 6;

    // Print header
    auto print_frame = [&]() {
        // Move cursor up N_AXES + 5 lines to overwrite previous output
        static bool first = true;
        if (!first) {
            std::printf("\033[%dA", N_AXES + 5);
        }
        first = false;

        std::printf("%-16s  %-23s  %6s\n", "Axis", "Value", "Raw");
        std::printf("%-16s  %-23s  %6s\n", "----", "-----", "---");

        for (int i = 0; i < N_AXES; ++i) {
            float v = gamepad.GetAxis(i);
            std::printf("%-16s  %s  %+6.3f\n",
                        AXIS_NAMES[i], bar(v).c_str(), v);
        }

        // MPC velocity commands derived from gamepad
        float vx      = -gamepad.GetAxis(1) * 0.5f;
        float yaw_cmd = -gamepad.GetAxis(3) * 0.5f;
        std::printf("\n");
        std::printf("  vx  (Axis1) = %+6.3f m/s    %s\n",
                    vx,  bar(vx / 0.5f).c_str());
        std::printf("  yaw (Axis3) = %+6.3f rad/s  %s\n",
                    yaw_cmd, bar(yaw_cmd / 0.5f).c_str());

        std::fflush(stdout);
    };

    // Print blank lines so the first overwrite doesn't scroll
    for (int i = 0; i < N_AXES + 5; ++i) std::printf("\n");

    while (g_running) {
        if (!gamepad.IsConnected()) {
            std::cerr << "[ERROR] Gamepad disconnected.\n";
            break;
        }
        print_frame();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));  // 20Hz display
    }

    std::printf("\n[Done]\n");
    return 0;
}
