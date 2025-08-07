#pragma once

#include <dds/dds.hpp>  
#include <dds/sub/ddssub.hpp>  

#define TOPIC_LOWCMD "rt/lowcmd"

namespace qi {

template<typename MSG>  
class DataReaderListener : public dds::sub::NoOpDataReaderListener<MSG> {  
public:  
    DataReaderListener(std::function<void(const MSG&)> handler) : handler_(handler) {}
    void on_data_available(dds::sub::DataReader<MSG>& reader) override {
        dds::sub::LoanedSamples<MSG> samples = reader.take();
        for (const auto& sample : samples) {
            if (sample.info().valid() && handler_) {
                handler_(sample.data());
            }
        }
    }
      
private:  
    std::function<void(const MSG&)> handler_;  
};  
  
template<typename MSG>  
class ChannelSubscriber {  
public:  
    explicit ChannelSubscriber(const std::string& topic_name,  
                              std::shared_ptr<dds::domain::DomainParticipant> participant)  
        : topic_name_(topic_name), participant_(participant), reader_(nullptr), listener_(nullptr) {  
    }  
  
    ~ChannelSubscriber() {  
        CloseChannel();  
    }  
  
    void InitChannel(const std::function<void(const MSG&)>& handler) {  
        try {  
            if (!participant_) {  
                std::cerr << "DDS participant not initialized" << std::endl;  
                return;  
            }  
  
            // 创建 Topic  
            topic_ = std::make_shared<dds::topic::Topic<MSG>>(*participant_, topic_name_);  
              
            // 创建 Subscriber  
            subscriber_ = std::make_shared<dds::sub::Subscriber>(*participant_);  
              
            // 创建 DataReader  
            // dds::sub::qos::DataReaderQos reader_qos = subscriber_->default_datareader_qos();
            // reader_qos << dds::core::policy::Reliability::Reliable()
            //            << dds::core::policy::History::KeepLast(1);
            //            // << dds::core::policy::Durability::TransientLocal();

            dds::sub::qos::DataReaderQos reader_qos = subscriber_->default_datareader_qos();
            reader_qos << dds::core::policy::Reliability::Reliable()  // 实时性更高，不保证数据不丢
                       << dds::core::policy::History::KeepLast(1); // 保留1个样本，降低内存开销
                       // << dds::core::policy::Durability::TransientLocal() // 保持不变
                       //  // 放宽截止时间
                       //  << dds::core::policy::Deadline(dds::core::Duration::from_millisecs(1.5))
                       //  // 强制低延时
                       //  << dds::core::policy::LatencyBudget(dds::core::Duration::from_millisecs(0.1));

            // 设置较高传输优先级(0~255)
            //reader_qos << dds::core::policy::TransportPriority(100);

            // reader_qos << dds::core::policy::ResourceLimits(
            //     1,          // 初始样本数：最小化内存预分配
            //     1,          // 最大样本数：只保留最新数据
            //     1           // 每个实例的最大样本数
            // );

            reader_ = std::make_shared<dds::sub::DataReader<MSG>>(*subscriber_, *topic_, reader_qos);  
              
            // 设置监听器  
            listener_ = std::make_shared<DataReaderListener<MSG>>(handler);  
            reader_->listener(listener_.get(), dds::core::status::StatusMask::data_available());  
              
            std::cout << "Subscriber initialized for topic: " << topic_name_ << std::endl;  
              
        } catch (const std::exception& e) {  
            std::cerr << "Exception in InitChannel: " << e.what() << std::endl;  
        }  
    }  
  
    void CloseChannel() {  
        if (reader_) {  
            reader_->listener(nullptr, dds::core::status::StatusMask::none());  
            reader_->close();  
            reader_.reset();  
        }  
        if (subscriber_) {  
            subscriber_->close();  
            subscriber_.reset();  
        }  
        if (topic_) {  
            topic_->close();  
            topic_.reset();  
        }  
        listener_.reset();  
        std::cout << "Subscriber closed for topic: " << topic_name_ << std::endl;  
    }  
  
    const std::string& GetChannelName() const {  
        return topic_name_;  
    }  
  
private:  
    std::string topic_name_;  
    std::shared_ptr<dds::domain::DomainParticipant> participant_;  
    std::shared_ptr<dds::topic::Topic<MSG>> topic_;  
    std::shared_ptr<dds::sub::Subscriber> subscriber_;  
    std::shared_ptr<dds::sub::DataReader<MSG>> reader_;  
    std::shared_ptr<DataReaderListener<MSG>> listener_;  
};  
  
template<typename MSG>  
using ChannelSubscriberPtr = std::shared_ptr<ChannelSubscriber<MSG>>;  
   

} // namespace qi