#pragma once
#include <ctime>
#include <cstdint>
#include <vector>
#include <memory>
#include <deque>
#include <thread>
#include <mutex>
#include <atomic>
#include <linux/joystick.h>
#include "robstride.hpp"

#define JS_AXIS_LIMIT 64

class ObsComponent {
public:
    virtual ~ObsComponent() = default;
    virtual std::vector<float> GetObs() const = 0;
    virtual void Update() = 0;
};

class Gamepad {
public:
    Gamepad(const char* dev = "/dev/input/js0");
    ~Gamepad();
    float GetAxis(int axis) const;
    bool IsConnected() const;

private:
    void ReadLoop();
    int fd;
    std::thread read_thread;
    std::atomic<bool> running;
    mutable std::mutex data_mutex;
    float axes[JS_AXIS_LIMIT]; // From linux/joystick.h
};

class RoboObsFrame {
public:
    std::vector<std::shared_ptr<ObsComponent>> components;

    RoboObsFrame();
    void AddComponent(std::shared_ptr<ObsComponent> component);
    std::vector<float> GetObs() const;

private:
    uint64_t timestamp;
};

class RoboObs {
public:
    int obs_dim;

    int history_length;
    std::deque<std::vector<float>> history;
    RoboObsFrame frame;

    RoboObs(int history_length);
    void AddComponent(std::shared_ptr<ObsComponent> component);
    void UpdateObs();
    std::vector<float> GetWholeObs() const;
    std::vector<float> GetSingleObs() const;
};

class IMUComponent : public ObsComponent {
public:
    IMUComponent(const char* dev);
    ~IMUComponent();
    std::vector<float> GetObs() const override;

    void Update() override;

    // Raw sensor accessors (used by mpc_cpp)
    float const* get_quaternion() const { return quaternion; }  // [w,x,y,z]
    float const* get_gyro()       const { return gyro; }        // [x,y,z] rad/s (WIT frame)
    float const* get_acc()        const { return acc; }         // [x,y,z] m/s^2 (WIT frame)

private:
    const char* dev_path;
    static IMUComponent* instance_;
    
    std::thread update_thread_;
    std::atomic<bool> running_;
    mutable std::mutex data_mutex_;
    void UpdateLoop();

    float acc[3];
    float gyro[3];
    float angle[3];
    float quaternion[4];

    void AutoScanSensor();
    static void SensorDataUpdata(uint32_t uiReg, uint32_t uiRegNum);
    static void Delayms(uint16_t ucMs);
    int ConfigureSensorOutputs(void);
    static void SerialWriteRegister(uint8_t *p_ucData, uint32_t uiLen);

    static int fd;
    static int s_iCurBaud;
    static constexpr int c_uiBaud[] = {2400 , 4800 , 9600 , 19200 ,
        38400 , 57600 , 115200 , 230400 , 460800 , 921600};
};

class JointComponent : public ObsComponent {
public:
    int joint_count;
    JointComponent(int joint_count, std::shared_ptr<RobstrideController> controller, std::vector<int>& motor_indices, const std::vector<float>& offsets) : 
        joint_count(joint_count), controller(controller), motor_indices(motor_indices), offsets(offsets) {}
    std::vector<float> GetObs() const override;
    void Update() override {}

private:
    std::shared_ptr<RobstrideController> controller;
    std::vector<int> motor_indices;
    std::vector<float> offsets;
};

class ActionComponent : public ObsComponent {
public:
    ActionComponent(int action_dim) : prev_actions(action_dim) {}
    ~ActionComponent() = default;
    std::vector<float> GetObs() const override;
    void SetAction(const std::vector<float>& action);
    void Update() override {}

private:
    std::vector<float> prev_actions;
};

class CommandComponent : public ObsComponent {
public:
    CommandComponent(int command_dim, std::shared_ptr<Gamepad> gamepad = nullptr) : 
        command(command_dim), gamepad(gamepad) {}
    ~CommandComponent() = default;
    std::vector<float> GetObs() const override;
    void SetCommand(const std::vector<float>& command);
    void Update() override;
private:
    mutable std::vector<float> command;
    std::shared_ptr<Gamepad> gamepad;
};

class RSDepthComponent : public ObsComponent {
public:
    RSDepthComponent(const char* dev, int width, int height);
    ~RSDepthComponent();
    std::vector<float> GetObs() const override;
    void Update() override {}
private:
    const char* dev_path;
    int width;
    int height;
    static RSDepthComponent* instance_;
    std::thread update_thread_;
    std::atomic<bool> running_;
    mutable std::mutex data_mutex_;
    void UpdateLoop();
};