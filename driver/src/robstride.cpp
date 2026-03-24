#include "robstride.hpp"
#include <cmath>
#include <cstring>
#include <iostream>

// Helper constants and structs
#define COMM_MIT            0x01
#define COMM_FEEDBACK       0x02
#define COMM_ENABLE         0x03
#define COMM_STOP           0x04
#define COMM_SET_ZERO       0x06
#define COMM_REPORT         0x18

#define P_MAX               12.57f
#define V_MAX               44.0f
#define T_MAX               17.0f
#define KP_MIN              0.0f
#define KP_MAX              500.0f
#define KD_MIN              0.0f
#define KD_MAX              5.0f

// Helper functions implementation
float RobstrideController::uint16_to_float(uint16_t x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (bits == 0) return 0.0f;
    return ((float)x * span / (float)((1 << bits) - 1)) + offset;
}

int RobstrideController::float_to_uint(float x, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (bits == 0) return 0;
    if (x > x_max) x = x_max;
    else if (x < x_min) x = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

// Static callback wrapper
static void robstride_can_rx_callback_wrapper(const struct device *dev, struct can_frame *frame, void *user_data) {
    RobstrideController* controller = static_cast<RobstrideController*>(user_data);
    if (controller) {
        controller->HandleCANMessage(dev, frame);
    }
}

void RobstrideController::HandleCANMessage(const struct device *dev, struct can_frame *frame) {
    (void)dev;
    // std::cout << "CAN RX: ID=" << frame->can_id << " DLC=" << (int)frame->can_dlc << std::endl;
    // CAN ID structure:
    // Bits 0-7: motor_id
    // Bits 8-15: master_id
    // Bits 16-23: reserved
    // Bits 24-28: msg_type (5 bits)
    uint32_t id = frame->can_id;
    if (frame->can_id & CAN_EFF_FLAG) {
        id = frame->can_id & CAN_EFF_MASK;
    }

    uint8_t motor_id = (id & 0xFF00) >> 8;
    // uint8_t master_id = (id >> 8) & 0xFF;
    uint8_t reserved = (id >> 16) & 0xFF;
    uint8_t msg_type = (id >> 24) & 0x1F;

    if (msg_type == COMM_FEEDBACK || msg_type == COMM_REPORT) {
        std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
        for (auto& motor : motor_data) {
            if (motor.motor_id == motor_id) {
                motor.online = true;
                motor.last_online_time = std::chrono::steady_clock::now();
                
                // Parse Error from reserved field (as per user snippet: data->err = (can_id->reserved) & 0x3f;)
                // int err = reserved & 0x3F;
                
                // Parse Data
                if (frame->len >= 8) {
                    uint16_t raw_pos = (frame->data[0] << 8) | frame->data[1];
                    uint16_t raw_vel = (frame->data[2] << 8) | frame->data[3];
                    uint16_t raw_tor = (frame->data[4] << 8) | frame->data[5];
                    // uint16_t raw_temp = (frame->data[6] << 8) | frame->data[7];

                    // Use limits from motor_info if available, or defaults
                    // The snippet uses fixed P_MAX, V_MAX, T_MAX for decoding
                    float p_max = P_MAX;
                    float v_max = motor.motor_info.max_speed > 0 ? motor.motor_info.max_speed : V_MAX;
                    float t_max = motor.motor_info.max_torque > 0 ? motor.motor_info.max_torque : T_MAX;

                    motor.state.position = uint16_to_float(raw_pos, -p_max, p_max, 16);
                    motor.state.velocity = uint16_to_float(raw_vel, -v_max, v_max, 16);
                    motor.state.torque = uint16_to_float(raw_tor, -t_max, t_max, 16);
                }
                break;
            }
        }
    }
}

RobstrideController::RobstrideController() : running(false) {
    can_rx_callback = robstride_can_rx_callback_wrapper;
    running = true;
    control_thread = std::thread([this]() {
        while(running) {
            auto now = std::chrono::steady_clock::now();
            {
                std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
                for (size_t i = 0; i < motor_data.size(); ++i) {
                    auto& motor = motor_data[i];
                    // Check for timeout (500ms)
                    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(now - motor.last_online_time).count();
                    if (duration > 500) {
                        motor.online = false;
                    }

                    // If offline but enabled, keep sending Enable command
                    if (!motor.online && motor.enabled && motor.offline_count++ == 10) {
                        motor.offline_count = 0;
                        // Re-send Enable command
                        // Note: EnableMotor just sets the flag and sends command once usually, 
                        // but here we want to re-trigger the send logic.
                        // We can call EnableMotor again, or extract the send logic.
                        // To avoid spamming too fast, we might want to rate limit this, 
                        // but the requirement says "keep sending". 10ms loop is reasonably fast.
                        EnableMotor(i); 
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    });
}

RobstrideController::~RobstrideController() {
    running = false;
    if (control_thread.joinable()) {
        control_thread.join();
    }
}

int RobstrideController::BindCAN(std::shared_ptr<CANInterface> can_interface) {
    if (!can_interface) return -1;
    can_interfaces.push_back(can_interface);
    return 0;
}

int RobstrideController::BindMotor(const char* can_if, std::unique_ptr<struct MotorInfo> motor_info) {
    if (!can_if || !motor_info) return -1;

    std::shared_ptr<CANInterface> target_iface = nullptr;
    for (auto& iface : can_interfaces) {
        if (std::string(iface->GetName()) == std::string(can_if)) {
            target_iface = iface;
            break;
        }
    }

    if (!target_iface) return -1;

    MotorData data;
    data.motor_info = *motor_info;
    data.can_iface = target_iface;
    data.motor_id = motor_info->motor_id;
    data.host_id = motor_info->host_id;
    data.enabled = false;
    data.online = false;
    data.last_online_time = std::chrono::steady_clock::now(); // Initialize to now so it doesn't timeout immediately if we want grace period, or 0 if we want immediate timeout.
    // Requirement says "500ms not received considered offline". 
    // Initial state is offline until first message? Or online?
    // Usually online=false initially.
    
    // Initialize default state/params
    data.state = {0.0f, 0.0f, 0.0f};
    data.mit_params = {25.0f, 0.5f, 0.0f, 0.0f};
    
    // Register filter
    CANInterface::can_filter filter;
    // Assuming motor_id is relevant for filtering. 

    // Construct filter ID to match the protocol structure
    // Bit layout (Little Endian):
    // Bits 0-7: motor_id
    // Bits 8-15: host_id (master_id)
    // Bits 16-23: reserved
    // Bits 24-28: msg_type
    
    uint32_t filter_id = 0;
    filter_id |= ((motor_info->motor_id & 0xFF) << 8);
    filter_id |= (motor_info->host_id & 0xFF);

    filter.id = filter_id;
    filter.mask = 0x0000FFFF; 
    filter.flags = CAN_FILTER_IDE; 
    
    target_iface->SetFilter(filter, can_rx_callback, this);

    {
        std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
        motor_data.push_back(data);
    }
    return motor_data.size() - 1;
}

struct motor_state RobstrideController::GetMotorState(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        return motor_data[motor_idx].state;
    }
    return {0, 0, 0};
}

int RobstrideController::SetMITParams(int motor_idx, struct MIT_params mit_params) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        motor_data[motor_idx].mit_params = mit_params;
        return 0;
    }
    return -1;
}

int RobstrideController::SendMITCommand(int motor_idx, float pos, float vel,
                                         float kp, float kd, float tau_ff) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        auto& motor = motor_data[motor_idx];

        // If kp/kd < 0 (default), use the globally configured MIT params
        float use_kp = (kp < 0.0f) ? motor.mit_params.kp : kp;
        float use_kd = (kd < 0.0f) ? motor.mit_params.kd : kd;

        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));

        // CAN ID layout:
        // Bits 0-7:   motor_id
        // Bits 8-23:  tau_ff (16-bit quantized, split across two bytes)
        // Bits 24-28: msg_type (COMM_MIT = 0x01)
        float t_max = motor.motor_info.max_torque > 0 ? motor.motor_info.max_torque : T_MAX;
        uint16_t tor_uint = float_to_uint(tau_ff, -t_max, t_max, 16);

        uint32_t id = 0;
        id |= (motor.motor_id & 0xFF);
        id |= ((tor_uint & 0xFF) << 8);
        id |= (((tor_uint >> 8) & 0xFF) << 16);
        id |= (COMM_MIT << 24);

        frame.can_id = id | CAN_EFF_FLAG;
        frame.can_dlc = 8;

        // Data bytes: [pos(16), vel(16), kp(16), kd(16)]
        float p_max = P_MAX;
        float v_max = motor.motor_info.max_speed > 0 ? motor.motor_info.max_speed : V_MAX;

        uint16_t pos_uint = float_to_uint(pos, -p_max, p_max, 16);
        uint16_t vel_uint = float_to_uint(vel, -v_max, v_max, 16);
        uint16_t kp_uint  = float_to_uint(use_kp, KP_MIN, KP_MAX, 16);
        uint16_t kd_uint  = float_to_uint(use_kd, KD_MIN, KD_MAX, 16);

        frame.data[0] = (pos_uint >> 8) & 0xFF;
        frame.data[1] = pos_uint & 0xFF;
        frame.data[2] = (vel_uint >> 8) & 0xFF;
        frame.data[3] = vel_uint & 0xFF;
        frame.data[4] = (kp_uint >> 8) & 0xFF;
        frame.data[5] = kp_uint & 0xFF;
        frame.data[6] = (kd_uint >> 8) & 0xFF;
        frame.data[7] = kd_uint & 0xFF;

        if (motor.can_iface) {
            motor.can_iface->SendMessage(&frame);
        }
        return 0;
    }
    return -1;
}

int RobstrideController::EnableMotor(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        auto& motor = motor_data[motor_idx];
        motor.enabled = true;
        
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        
        uint32_t id = 0;
        id |= (motor.motor_id & 0xFF);
        id |= ((motor.host_id & 0xFF) << 8);
        id |= (COMM_ENABLE << 24);
        
        frame.can_id = id | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        
        if (motor.can_iface) {
            motor.can_iface->SendMessage(&frame);
        }
        return 0;
    }
    return -1;
}

int RobstrideController::DisableMotor(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        auto& motor = motor_data[motor_idx];
        motor.enabled = false;
        
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        
        uint32_t id = 0;
        id |= (motor.motor_id & 0xFF);
        id |= ((motor.host_id & 0xFF) << 8);
        id |= (COMM_STOP << 24);
        
        frame.can_id = id | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        
        if (motor.can_iface) {
            motor.can_iface->SendMessage(&frame);
        }
        return 0;
    }
    return -1;
}

int RobstrideController::EnableAutoReport(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        auto& motor = motor_data[motor_idx];
        
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        
        uint32_t id = 0;
        id |= (motor.motor_id & 0xFF);
        id |= ((motor.host_id & 0xFF) << 8);
        id |= (COMM_REPORT << 24);
        
        frame.can_id = id | CAN_EFF_FLAG;
        frame.can_dlc = 8;

        frame.data[0] = 0x01;
		frame.data[1] = 0x02;
		frame.data[2] = 0x03;
		frame.data[3] = 0x04;
		frame.data[4] = 0x05;
		frame.data[5] = 0x06;
		frame.data[6] = 0x01;
		frame.data[7] = 0x00;
        
        if (motor.can_iface) {
            motor.can_iface->SendMessage(&frame);
        }
        return 0;
    }
    return -1;
}

int RobstrideController::DisableAutoReport(int motor_idx) {
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        auto& motor = motor_data[motor_idx];
        
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        
        uint32_t id = 0;
        id |= (motor.motor_id & 0xFF);
        id |= ((motor.host_id & 0xFF) << 8);
        id |= (COMM_REPORT << 24);
        
        frame.can_id = id | CAN_EFF_FLAG;
        frame.can_dlc = 8;

        frame.data[0] = 0x01;
		frame.data[1] = 0x02;
		frame.data[2] = 0x03;
		frame.data[3] = 0x04;
		frame.data[4] = 0x05;
		frame.data[5] = 0x06;
		frame.data[6] = 0x00;
		frame.data[7] = 0x00;
        
        if (motor.can_iface) {
            motor.can_iface->SendMessage(&frame);
        }
        return 0;
    }
    return -1;
}

int RobstrideController::SetZero(int motor_idx){
    std::lock_guard<std::recursive_mutex> lock(motor_data_mutex);
    if (motor_idx >= 0 && (size_t)motor_idx < motor_data.size()) {
        auto& motor = motor_data[motor_idx];      
        struct can_frame frame;
        std::memset(&frame, 0, sizeof(frame));
        
        uint32_t id = 0;
        id |= (motor.motor_id & 0xFF);
        id |= ((motor.host_id & 0xFF) << 8);
        id |= (COMM_SET_ZERO << 24);
        
        frame.can_id = id | CAN_EFF_FLAG;
        frame.can_dlc = 8;
        
        frame.data[0] = 0x01;

        if (motor.can_iface) {
            motor.can_iface->SendMessage(&frame);
        }
        return 0;
    }
    return -1;
}