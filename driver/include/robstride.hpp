#pragma once
#include <memory>
#include <vector>
#include <cstring>
#include "can_interface.hpp"
#include <mutex>
#include <atomic>
#include <thread>

struct motor_state {
    float position;
    float velocity;
    float torque;
};

struct MIT_params {
    float kp;
    float kd;

    float vel_limit;
    float torque_limit;
};

class RobstrideController {
public:
    RobstrideController();
    ~RobstrideController();
    int BindCAN(std::shared_ptr<CANInterface> can_interface);

    struct MotorInfo {
        int motor_id;
        int host_id;

        float max_torque;
        float max_speed;
        float max_kp;
        float max_kd;
    };
    int BindMotor(const char* can_if, std::unique_ptr<struct MotorInfo> motor_info);
    struct motor_state GetMotorState(int motor_idx);
    int SetMITParams(int motor_idx, struct MIT_params mit_params);
    // Full MIT mode command: τ_motor = kp*(pos-q) + kd*(vel-dq) + tau_ff
    int SendMITCommand(int motor_idx, float pos, float vel = 0.0f,
                       float kp = -1.0f, float kd = -1.0f, float tau_ff = 0.0f);
    int EnableMotor(int motor_idx);
    int DisableMotor(int motor_idx);
    int EnableAutoReport(int motor_idx);
    int DisableAutoReport(int motor_idx);
    int SetZero(int motor_idx);

    // Callback for CAN RX
    void HandleCANMessage(const struct device *dev, struct can_frame *frame);

private:
    // 通信类型
    const uint32_t COMM_OPERATION_CONTROL = 1;
    const uint32_t COMM_ENABLE = 3;
    const uint32_t COMM_WRITE_PARAMETER = 18;

    // 参数 ID
    const uint16_t PARAM_MODE = 0x7005;
    const uint16_t PARAM_VELOCITY_LIMIT = 0x7017;
    const uint16_t PARAM_TORQUE_LIMIT = 0x700B;

    // Callback for CAN RX
    // void HandleCANMessage(const struct device *dev, struct can_frame *frame);

    float uint16_to_float(uint16_t x, float x_min, float x_max, int bits);
    int float_to_uint(float x, float x_min, float x_max, int bits);
    
    struct MotorData {
        struct MotorInfo motor_info;
        std::shared_ptr<CANInterface> can_iface;
        int motor_id;
        int host_id;
        bool enabled;
        bool online;
        std::chrono::steady_clock::time_point last_online_time;

        struct MIT_params mit_params;
        struct motor_state state;
        int offline_count;
    };
    std::vector<std::shared_ptr<CANInterface>> can_interfaces;
    std::vector<struct MotorData> motor_data;

    CANInterface::can_rx_callback_t can_rx_callback;

    std::thread control_thread;
    std::atomic<bool> running;
    std::recursive_mutex motor_data_mutex;
};
