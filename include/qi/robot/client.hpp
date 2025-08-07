#pragma once
  
#include "RobotState.hpp"  
#include "RobotCmd.hpp"  
#include "qi/common/dds/channel_publisher.hpp"
#include "qi/common/dds/channel_subscriber.hpp"  
  
namespace qi {  
  
class RobotClient {  
public:  
    RobotClient();  
    ~RobotClient();  
      
    bool Init(const std::string& network_interface = "lo");  
    bool SendCommand(const RobotCmd& cmd);  
    bool GetState(RobotState& state);  
      
    void SetStateCallback(const std::function<void(const RobotState&)>& callback);  
      
private:  
    ChannelPublisherPtr<RobotCmd> cmd_publisher_;  
    ChannelSubscriberPtr<RobotState> state_subscriber_;  
    RobotState latest_state_;  
    bool initialized_;   

    std::mutex state_mutex_;  
    std::function<void(const RobotState&)> state_callback_;  

    std::shared_ptr<dds::domain::DomainParticipant> participant_;  
};  
  
}