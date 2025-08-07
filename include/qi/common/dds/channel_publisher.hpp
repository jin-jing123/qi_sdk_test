#pragma once

#include <dds/dds.hpp>  
#include <dds/pub/ddspub.hpp>  

#define TOPIC_LOWSTATE "rt/lowstate"

namespace qi {

template<typename MSG>  
class ChannelPublisher {  
public:  
    explicit ChannelPublisher(const std::string& topic_name,   
                             std::shared_ptr<dds::domain::DomainParticipant> participant)  
        : topic_name_(topic_name), participant_(participant), writer_(nullptr) {  
    }  
  
    ~ChannelPublisher() {  
        CloseChannel();  
    }  
  
    bool InitChannel() {  
        try {  
            if (!participant_) {  
                std::cerr << "DDS participant not initialized" << std::endl;  
                return false;  
            }  
  
            // 创建 Topic  
            topic_ = std::make_shared<dds::topic::Topic<MSG>>(*participant_, topic_name_);  
              
            // 创建 Publisher  
            publisher_ = std::make_shared<dds::pub::Publisher>(*participant_);  
              
            // 创建 DataWriter  
            // dds::pub::qos::DataWriterQos writer_qos = publisher_->default_datawriter_qos();
            // writer_qos << dds::core::policy::Reliability::Reliable()
            //            << dds::core::policy::History::KeepLast(10)
            //            << dds::core::policy::Durability::TransientLocal();

            dds::pub::qos::DataWriterQos writer_qos = publisher_->default_datawriter_qos();

            // writer_qos << dds::core::policy::Reliability::Reliable()  // 实时性更高，不保证数据不丢
            //            << dds::core::policy::History::KeepLast(1) // 保留1个样本，降低内存开销
            //            << dds::core::policy::Durability::TransientLocal() // 保持不变
            //             // 放宽截止时间
            //             << dds::core::policy::Deadline(dds::core::Duration::from_millisecs(1.5))
            //             // 强制低延时
            //             << dds::core::policy::LatencyBudget(dds::core::Duration::from_millisecs(0.1))
            //             // 设置Lifespan为1.5ms（1500000纳秒）
            //            << dds::core::policy::Lifespan(dds::core::Duration(0, 1500000));

            // // 设置较高传输优先级(0~255)
            // writer_qos << dds::core::policy::TransportPriority(100);

            // CycloneDDS 特定优化
            // writer_qos << dds::core::policy::Property(
            //     // 禁用批处理，立即发送每个样本
            //     {"dds.datawriter.unicast.batching", "false"},
            //     {"dds.datawriter.multicast.batching", "false"},
            //
            //     // 其他网络优化
            //     {"dds.transport.tcp.nodelay", "true"},           // 禁用 Nagle 算法
            //     {"dds.transport.udp.ipv4.max_message_size", "9000"}  // 支持 Jumbo Frame
            // );

            // 设置零发送方资源限制（避免因队列满导致的延迟）
            // writer_qos << dds::core::policy::ResourceLimits(
            //     1,          // 初始样本数
            //     1,          // 最大样本数
            //     1);         // 每个实例的最大样本数

            writer_qos << dds::core::policy::Reliability::Reliable()  // 实时性更高，不保证数据不丢
                       << dds::core::policy::History::KeepLast(1);

            writer_ = std::make_shared<dds::pub::DataWriter<MSG>>(*publisher_, *topic_, writer_qos);  
              
            std::cout << "Publisher initialized for topic: " << topic_name_ << std::endl;  
            return true;  
        } catch (const std::exception& e) {  
            std::cerr << "Exception in InitChannel: " << e.what() << std::endl;  
            return false;  
        }  
    }  
  
    bool Write(const MSG& message) {  
        if (!writer_) {  
            std::cerr << "Publisher not initialized" << std::endl;  
            return false;  
        }  
  
        try {  
            writer_->write(message);  
            return true;  
        } catch (const std::exception& e) {  
            std::cerr << "Failed to write message: " << e.what() << std::endl;  
            return false;  
        }  
    }  
  
    void CloseChannel() {  
        if (writer_) {  
            writer_->close();  
            writer_.reset();  
        }  
        if (publisher_) {  
            publisher_->close();  
            publisher_.reset();  
        }  
        if (topic_) {  
            topic_->close();  
            topic_.reset();  
        }  
        std::cout << "Publisher closed for topic: " << topic_name_ << std::endl;  
    }  
  
private:  
    std::string topic_name_;  
    std::shared_ptr<dds::domain::DomainParticipant> participant_;  
    std::shared_ptr<dds::topic::Topic<MSG>> topic_;  
    std::shared_ptr<dds::pub::Publisher> publisher_;  
    std::shared_ptr<dds::pub::DataWriter<MSG>> writer_;  
};  
  
template<typename MSG>  
using ChannelPublisherPtr = std::shared_ptr<ChannelPublisher<MSG>>;  
  
 
}

