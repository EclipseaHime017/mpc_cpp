#pragma once
#include <linux/can.h>
#include <cstdint>
#include <vector>
#include <thread>
#include <atomic>
#include <string>
#include <mutex>

#ifndef BIT
#define BIT(n)  (1UL << (n))
#endif

/** Filter matches frames with extended (29-bit) CAN IDs */
#define CAN_FILTER_IDE  BIT(0)

class CANInterface {
public:
    typedef void (*can_rx_callback_t)(const struct device *dev, struct can_frame *frame,
        void *user_data);
    struct can_filter {
        /** CAN identifier to match. */
        uint32_t id;
        /** CAN identifier matching mask. If a bit in this mask is 0, the value
        * of the corresponding bit in the ``id`` field is ignored by the filter.
        */
        uint32_t mask;
        /** Flags. @see @ref CAN_FILTER_FLAGS. */
        uint8_t flags;
    };
    CANInterface(const char* can_if);
    ~CANInterface();
    const char* GetName() const;
    int SendMessage(const struct can_frame* frame);
    int SetFilter(struct can_filter filter, can_rx_callback_t callback, void *user_data);
private:
    int can_socket;
    struct can_filter_info {
        struct can_filter filter;
        can_rx_callback_t callback;
        void *user_data;
    };
    std::vector<struct can_filter_info> filter_info;
    std::string if_name;
    std::thread can_thread;
    std::atomic<bool> running;
    std::mutex filter_mutex;
};