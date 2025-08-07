#include <iostream>
#include <atomic>
#include <array>
#include <thread>
#include <chrono>

template<typename T, size_t N>
class DoubleBuffer {
public:
    void write(const T& data) {
        // 写入当前活跃缓冲区
        buffer[write_idx][write_pos++] = data;

        // 如果缓冲区满，切换到另一个缓冲区
        if (write_pos >= N) {
            write_pos = 0;
            active_buffer.store(write_idx, std::memory_order_release); // 发布新数据
            write_idx = 1 - write_idx; // 切换缓冲区
        }
    }

    bool read(T& data) {
        int current_active = active_buffer.load(std::memory_order_acquire);

        // 如果当前缓冲区有数据可读
        if (read_pos < N) {
            data = buffer[current_active][read_pos++];
            return true;
        }

        // 如果当前缓冲区读完，尝试切换到另一个缓冲区
        if (current_active != read_idx) {
            read_idx = current_active;
            read_pos = 0;
            if (read_pos < N) {
                data = buffer[read_idx][read_pos++];
                return true;
            }
        }

        return false; // 没有新数据
    }

private:
    std::array<std::array<T, N>, 2> buffer; // 双缓冲区
    std::atomic<int> active_buffer{0};      // 当前活跃缓冲区（0 或 1）
    int write_idx = 0;                      // A 线程当前写入的缓冲区
    int write_pos = 0;                      // A 线程当前写入位置
    int read_idx = 0;                       // B 线程当前读取的缓冲区
    int read_pos = N;                       // B 线程当前读取位置（初始设为 N，表示无数据）
};