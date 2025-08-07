//
// Created by coral on 25-4-9.
//

// 这个头文件和erthercat 进程共享

#pragma once
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <memory>
#include <cstring>
#include <iostream>
#include <type_traits>
using namespace boost::interprocess;
using ShareMemory       = boost::interprocess::shared_memory_object;
using ShareMemoryRegion = boost::interprocess::mapped_region;
using ShareMemoryMutex  = boost::interprocess::named_mutex;
template <typename T>
class SharedMemoryIPC {
public:
    explicit SharedMemoryIPC(std::string channel, std::size_t memory_size = sizeof(T))
            : channel_(std::move(channel)), memory_size_(memory_size) {
        // 静态断言确保类型安全
        static_assert(std::is_trivially_copyable_v<T>,
            "SharedMemoryIPC requires trivially copyable types");
        // 尝试打开现有共享内存
        try {
            creator_ = false;
            sharedMemoryPtr_ = std::make_unique<ShareMemory>(open_only, channel_.c_str(), read_write);
            std::cout << "open..." << std::endl;
        }
        catch (const interprocess_exception&) {
            // 不存在则创建
            creator_ = true;
            sharedMemoryPtr_ = std::make_unique<ShareMemory>(
                open_or_create, channel_.c_str(), read_write);
            sharedMemoryPtr_->truncate(memory_size_);
            std::cout << "create..." << std::endl;
        }

        // 映射内存区域
        regionPtr_ = std::make_unique<ShareMemoryRegion>(*sharedMemoryPtr_, read_write);
        // 如果是创建者，初始化内存
        if (creator_) {
            std::memset(regionPtr_->get_address(), 0, memory_size_);
        }
        // 创建/打开互斥锁
        std::string mux_name = channel_ + "_mux";
        mutexPtr_ = std::make_unique<ShareMemoryMutex>(open_or_create, mux_name.c_str());
    }
    // 保持原有接口
    void write(const T& data) {
        scoped_lock<ShareMemoryMutex> lock(*mutexPtr_);  // RAII锁
        std::memcpy(regionPtr_->get_address(), &data, memory_size_);
    }
    // 保持原有接口
    T read() {
        scoped_lock<ShareMemoryMutex> lock(*mutexPtr_);  // RAII锁
        T result;
        std::memcpy(&result, regionPtr_->get_address(), memory_size_);
        return result;
    }
    ~SharedMemoryIPC() {
        if (creator_) {
            // 确保互斥锁被持有后再清理
            scoped_lock<ShareMemoryMutex> lock(*mutexPtr_);
            sharedMemoryPtr_->remove(channel_.c_str());
            std::string mux_name = channel_ + "_mux";
            ShareMemoryMutex::remove(mux_name.c_str());
        }
    }
private:
    std::string channel_;
    std::size_t memory_size_;
    bool creator_;
    std::unique_ptr<ShareMemory> sharedMemoryPtr_;
    std::unique_ptr<ShareMemoryRegion> regionPtr_;
    std::unique_ptr<ShareMemoryMutex> mutexPtr_;
};
