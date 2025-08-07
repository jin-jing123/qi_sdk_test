#pragma once
  
#include "SNState.hpp"  
#include "LowState_.hpp"  
#include "IMUState_.hpp"
#include "LowCmd_.hpp"
#include "qi/common/dds/channel_publisher.hpp"
#include "qi/common/dds/channel_subscriber.hpp"  

using LowCmd = qi::msg::dds_::LowCmd_;
using MotorCmd = qi::msg::dds_::MotorCmd_;
using LowState = qi::msg::dds_::LowState_;
using IMUState = qi::msg::dds_::IMUState_;
using MotorState = qi::msg::dds_::MotorState_;

namespace qi {  
  
class SNClient {  
public:  
    SNClient();  
    ~SNClient();  
      
    bool Init(const std::string& network_interface = "lo");  
    bool SendCommand(const LowCmd& cmd);  
    bool GetState(LowState& state);  
    bool GetIMUState(IMUState& state);
      
    void SetStateCallback(const std::function<void(const LowState&)>& callback);  
    void SetIMUStateCallback(const std::function<void(const IMUState&)>& callback);

    void AnkleInverseKinematics(LowCmd& cmd);

private:  
    ChannelPublisherPtr<LowCmd> cmd_publisher_;  
    ChannelSubscriberPtr<LowState> state_subscriber_;  
    ChannelSubscriberPtr<IMUState> imu_state_subscriber_;
    LowState latest_state_;  
    IMUState latest_imu_state_;
    bool initialized_;   

    std::mutex state_mutex_;  
    std::mutex imu_state_mutex_;
    std::function<void(const LowState&)> state_callback_;  
    std::function<void(const IMUState&)> imu_state_callback_;

    std::shared_ptr<dds::domain::DomainParticipant> participant_;  

};  
  
}