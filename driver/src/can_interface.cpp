#include "can_interface.hpp"
#include <cstdio>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <mutex>

CANInterface::CANInterface(const char* can_if) : running(false), can_socket(-1), if_name(can_if) {
    struct sockaddr_can addr;
    struct ifreq ifr;

    if ((can_socket = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Socket");
        return;
    }

    std::strncpy(ifr.ifr_name, can_if, IFNAMSIZ - 1);
    if (ioctl(can_socket, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        close(can_socket);
        can_socket = -1;
        return;
    }

    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(can_socket, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Bind");
        close(can_socket);
        can_socket = -1;
        return;
    }

    // Non-blocking writes: if the TX queue is full (e.g. bus-off with no ACK),
    // write() returns EAGAIN instead of blocking indefinitely.
    int flags = fcntl(can_socket, F_GETFL, 0);
    fcntl(can_socket, F_SETFL, flags | O_NONBLOCK);

    running = true;
    can_thread = std::thread([this]() {
        struct can_frame frame;
        while (running) {
            // Blocking read
            int nbytes = read(can_socket, &frame, sizeof(struct can_frame));

            if (nbytes < 0) {
                // When socket is closed in destructor, read returns -1
                if (errno != EAGAIN && errno != EBADF) {
                    // perror("Read"); 
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10)); 
                continue;
            }

            if (nbytes < (int)sizeof(struct can_frame)) {
                continue;
            }

            // Dispatch
            {
                std::lock_guard<std::mutex> lock(filter_mutex);
                for (const auto& info : filter_info) {
                    bool match = true;
                    
                    // 1. Check IDE (Extended Frame) matches filter flag
                    bool is_frame_extended = (frame.can_id & CAN_EFF_FLAG);
                    bool is_filter_extended = (info.filter.flags & CAN_FILTER_IDE);
                    
                    // If mask is 0, we treat it as "match all", ignoring flags
                    if (info.filter.mask != 0 && (is_frame_extended != is_filter_extended)) {
                        match = false;
                    }

                    // 2. Mask logic: (id & mask) == (filter_id & mask)
                    if (match) {
                        // Remove EFF/RTR/ERR flags for ID comparison
                        uint32_t clean_id = frame.can_id & CAN_EFF_MASK; 
                        if ((clean_id & info.filter.mask) != (info.filter.id & info.filter.mask)) {
                            match = false;
                        }
                    }
                    
                    if (match && info.callback) {
                        info.callback(nullptr, &frame, info.user_data);
                    }
                }
            }
        }
    });
}

CANInterface::~CANInterface() {
    running = false;
    if (can_socket >= 0) {
        // Closing socket unblocks the read call
        close(can_socket);
        can_socket = -1;
    }
    if (can_thread.joinable()) {
        can_thread.join();
    }
}

const char* CANInterface::GetName() const {
    return if_name.c_str();
}

int CANInterface::SendMessage(const struct can_frame* frame) {
    if (can_socket < 0) return -1;
    
    int nbytes = write(can_socket, frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        // EAGAIN/EWOULDBLOCK/ENOBUFS: TX queue full (bus-off or no ACK) — skip silently.
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != ENOBUFS)
            perror("CAN write");
        return -1;
    }
    return 0;
}

int CANInterface::SetFilter(struct can_filter filter, can_rx_callback_t callback, void *user_data) {
    std::lock_guard<std::mutex> lock(filter_mutex);
    struct can_filter_info info;
    info.filter = filter;
    info.callback = callback;
    info.user_data = user_data;
    filter_info.push_back(info);
    return 0;
}
