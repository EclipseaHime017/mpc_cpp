#include "observations.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <cmath>
#include <fcntl.h>

#include "serial.h"
#include "wit_c_sdk.h"

/*
    Observations:
    Angular Velocity: 3
    Projected Gravity: 3
    Commands: 3
    Joint Positions: 12
    Joint Velocities: 12
    Previous Actions: 12

    Total: 45
*/

IMUComponent* IMUComponent::instance_ = nullptr;
int IMUComponent::fd = -1;
int IMUComponent::s_iCurBaud = 9600;

static volatile char s_cDataUpdate = 0;

IMUComponent::IMUComponent(const char* dev) : dev_path(dev), running_(true) {
    instance_ = this;

    std::memset(acc, 0, sizeof(acc));
    std::memset(gyro, 0, sizeof(gyro));
    std::memset(angle, 0, sizeof(angle));
    std::memset(quaternion, 0, sizeof(quaternion));

    WitInit(WIT_PROTOCOL_NORMAL, 0x50);
    WitDelayMsRegister(Delayms);
    WitSerialWriteRegister(SerialWriteRegister);
    WitRegisterCallBack(SensorDataUpdata);

    AutoScanSensor();

    if (ConfigureSensorOutputs() != 0) {
        std::cerr << "Configure IMU failed" << std::endl;
    }

    update_thread_ = std::thread(&IMUComponent::UpdateLoop, this);
}

IMUComponent::~IMUComponent() {
    running_ = false;
    if (update_thread_.joinable()) {
        update_thread_.join();
    }

    if (fd >= 0) {
        serial_close(fd);
    }
    if (instance_ == this) {
        instance_ = nullptr;
    }
}

void IMUComponent::UpdateLoop() {
    while (running_) {
        Update();
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
}

void IMUComponent::Update() {
    if (fd < 0) return;
    
    char cBuff[1];
    while(serial_read_data(fd, (unsigned char*)cBuff, 1)) {
        WitSerialDataIn(cBuff[0]);
    }
}

std::vector<float> IMUComponent::GetObs() const {
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<float> obs;
    obs.reserve(7);

    // 如果以 IMU 的 Y 轴为前向、-X 轴为左向、Z 轴为上向
    obs.push_back(gyro[1] * M_PI / 180.0f);   // 传给 policy 的前向: IMU_Y
    obs.push_back(-gyro[0] * M_PI / 180.0f);  // 传给 policy 的左向: -IMU_X
    obs.push_back(gyro[2] * M_PI / 180.0f);   // 传给 policy 的上向: IMU_Z

    float quat0 = quaternion[0];
    float quat1 = quaternion[1];
    float quat2 = quaternion[2];
    float quat3 = quaternion[3];

    float gx = 2 * (quat1 * quat3 - quat0 * quat2); // IMU X轴投影重力 (物理右向)
    float gy = 2 * (quat2 * quat3 + quat0 * quat1); // IMU Y轴投影重力 (物理前向)
    float gz = 1 - 2 * (quat1 * quat1 + quat2 * quat2);

    float norm = std::sqrt(gx*gx + gy*gy + gz*gz);
    if (norm > 1e-6f) {
        gx /= norm;
        gy /= norm;
        gz /= norm;
    }
    
    // 同样，把前、左、上的重力投影传给 Policy
    // 原来是: -gx (前), -gy(左), -gz(上)
    obs.push_back(-gy);  // 新的前向是 Y    -> 传入 -gy
    obs.push_back(gx);   // 新的左向是 -X   -> 传入 -(-gx) = gx
    obs.push_back(-gz);  // Z 轴不变        -> 传入 -gz
    
    return obs;
}

void IMUComponent::AutoScanSensor() {
    int i, iRetry;
    char cBuff[1];
    
    for(i = 0; i < sizeof(c_uiBaud)/sizeof(int); i++) {
        if(fd >= 0) serial_close(fd);
        
        s_iCurBaud = c_uiBaud[i];
        fd = serial_open((unsigned char*)dev_path, s_iCurBaud);
        
        if(fd < 0) continue;

        iRetry = 2;
        do {
            s_cDataUpdate = 0;
            WitReadReg(AX, 3);
            Delayms(200);
            
            while(serial_read_data(fd, (unsigned char*)cBuff, 1)) {
                WitSerialDataIn(cBuff[0]);
            }
            
            if(s_cDataUpdate != 0) {
                std::cout << "IMU Connected at baud " << s_iCurBaud << std::endl;
                return;
            }
            iRetry--;
        } while(iRetry);
    }
    std::cerr << "Can not find IMU sensor" << std::endl;
}

int IMUComponent::ConfigureSensorOutputs() {
    // 设置回传内容：加速度、角速度、四元数
    int32_t ret = WitSetContent(RSW_ACC | RSW_GYRO | RSW_Q);
    if(ret != WIT_HAL_OK) {
        return -1;
    }
    return 0;
}

void IMUComponent::SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum) {
    s_cDataUpdate = 1; // 标记收到数据，用于 AutoScan
    
    if (!instance_) return;

    std::lock_guard<std::mutex> lock(instance_->data_mutex_);

    for(int i = 0; i < uiRegNum; i++) {
        switch(uiReg) {
            case AX: case AY: case AZ:
                for(int j=0; j<3; j++) instance_->acc[j] = sReg[AX+j] / 32768.0f * 16.0f;
                break;
            case GX: case GY: case GZ:
                for(int j=0; j<3; j++) instance_->gyro[j] = sReg[GX+j] / 32768.0f * 2000.0f;
                break;
            case Roll: case Pitch: case Yaw:
                for(int j=0; j<3; j++) instance_->angle[j] = sReg[Roll+j] / 32768.0f * 180.0f;
                break;
            case q0: case q1: case q2: case q3:
                for(int j=0; j<4; j++) instance_->quaternion[j] = sReg[q0+j] / 32768.0f;
                break;
        }
        uiReg++;
    }
}

void IMUComponent::Delayms(uint16_t ucMs) {
    usleep(ucMs * 1000);
}

void IMUComponent::SerialWriteRegister(uint8_t *p_ucData, uint32_t uiLen) {
    if (fd >= 0) {
        serial_write_data(fd, p_ucData, uiLen);
    }
}

std::vector<float> JointComponent::GetObs() const {
    std::vector<float> obs(joint_count * 2);
    if (motor_indices.size() != (size_t)joint_count) {
        std::cerr << "JointComponent: controllers size mismatch" << std::endl;
        return obs;
    }
    int idx = 0;
    for (int motor_idx : motor_indices) {
        auto state = controller->GetMotorState(motor_idx);

        // obs[idx] = state.position;


        // For knee joints (indices 8-11) the motor has a gear reduction.
        // Convert motor shaft angle to joint angle by dividing by the gear ratio.
        float pos = state.position;
        float vel = state.velocity;
         // Subtract joint offsets to get observations in relative coordinates
        if (idx >= 8 && idx <= 11) {
            obs[idx] = (pos - offsets[idx]) / 1.667f;
            obs[joint_count + idx] = vel / 1.667f;

        }
        else{
            obs[idx] = pos - offsets[idx];
            obs[joint_count + idx] = vel;
        }
        idx++;
    }
    return obs;
}

// Implementations for ActionComponent, CommandComponent, RoboObsFrame, RoboObs

Gamepad::Gamepad(const char* dev) : fd(-1), running(true) {
    fd = open(dev, O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
        std::cerr << "Failed to open gamepad device: " << dev << std::endl;
        running = false;
        return;
    }
    std::memset(axes, 0, sizeof(axes));
    read_thread = std::thread(&Gamepad::ReadLoop, this);
}

Gamepad::~Gamepad() {
    running = false;
    if (read_thread.joinable()) {
        read_thread.join();
    }
    if (fd >= 0) {
        close(fd);
    }
}

float Gamepad::GetAxis(int axis) const {
    if (axis < 0 || axis >= JS_AXIS_LIMIT) return 0.0f;
    std::lock_guard<std::mutex> lock(data_mutex);
    return axes[axis];
}

bool Gamepad::IsConnected() const {
    return fd >= 0;
}

void Gamepad::ReadLoop() {
    struct js_event event;
    while (running) {
        bool any_read = false;
        while (read(fd, &event, sizeof(event)) == sizeof(event)) {
            any_read = true;
            if (event.type & JS_EVENT_AXIS) {
                if (event.number < JS_AXIS_LIMIT) {
                    std::lock_guard<std::mutex> lock(data_mutex);
                    axes[event.number] = (float)event.value / 32767.0f;
                }
            } else if (event.type & JS_EVENT_BUTTON) {
                std::cout << "[Gamepad] Button " << (int)event.number 
                          << " State: " << (int)event.value << std::endl;
            }
        }
        if (!any_read && fd >= 0) {
            // No data available
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

std::vector<float> ActionComponent::GetObs() const {
    return prev_actions;
}

void ActionComponent::SetAction(const std::vector<float>& action) {
    prev_actions = action;
}

std::vector<float> CommandComponent::GetObs() const {
    if (gamepad && gamepad->IsConnected()) {
        // axis1 [-32767,32767] 映射到 [1.0,-1.0] -> command[0]
        // axis0 映射到 command[1]
        // axis3 映射到 command[2]
        command[0] = -gamepad->GetAxis(1) * 0.5;
        command[1] = -gamepad->GetAxis(0) * 0;
        command[2] = -gamepad->GetAxis(3) * 0.5;
    }
    return command;
}

void CommandComponent::SetCommand(const std::vector<float>& cmd) {
    command = cmd;
}

void CommandComponent::Update() {
    // Gamepad is updated in its own thread
}

RoboObsFrame::RoboObsFrame() : timestamp(0) {}

void RoboObsFrame::AddComponent(std::shared_ptr<ObsComponent> component) {
    components.push_back(component);
}

std::vector<float> RoboObsFrame::GetObs() const {
    std::vector<float> obs;
    for (const auto& comp : components) {
        auto sub_obs = comp->GetObs();
        obs.insert(obs.end(), sub_obs.begin(), sub_obs.end());
    }
    return obs;
}

RoboObs::RoboObs(int history_length) : obs_dim(0), history_length(history_length) {}

void RoboObs::AddComponent(std::shared_ptr<ObsComponent> component) {
    frame.AddComponent(component);
}

void RoboObs::UpdateObs() {
    for (const auto& component : frame.components) {
        component->Update();
    }
    auto current_obs = frame.GetObs();
    obs_dim = current_obs.size();
    
    history.push_back(current_obs);

    while (history.size() > (size_t)history_length) {
        history.pop_front();
    }
}

std::vector<float> RoboObs::GetWholeObs() const {
    std::vector<float> whole_obs;
    int missing = history_length - history.size();
    
    if (missing > 0 && obs_dim > 0) {
        std::vector<float> padding(missing * obs_dim, 0.0f);
        whole_obs.insert(whole_obs.end(), padding.begin(), padding.end());
    }
    
    for (const auto& obs : history) {
        whole_obs.insert(whole_obs.end(), obs.begin(), obs.end());
    }
    return whole_obs;
}

std::vector<float> RoboObs::GetSingleObs() const {
    if (history.empty()) return std::vector<float>(obs_dim, 0.0f);
    return history.back();
}

