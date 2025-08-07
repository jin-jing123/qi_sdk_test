#include <iostream>
#include "sn_client.hpp"
#include "LowState.hpp"
#include "IMUState.hpp"

using namespace qi;

int main() {
    // 创建 SNClient 实例
    SNClient snClient;

    // 初始化 SNClient
    if (!snClient.Init("lo")) {
        std::cerr << "Failed to initialize SNClient" << std::endl;
        return 1;
    }


    // 设置状态回调函数  
    snClient.SetStateCallback([](const qi::msg::dds_::LowState_& state) {  
        static int counter = 0;  
        if (++counter % 100 == 0) {  // 每100次打印一次，避免输出过多
            std::cout << "Received shennong robot state - timestamp: " << state.timestamp()   
                      << ", mode: " << state.mode() << std::endl;  
             
            // 打印IMU状态
            for (size_t i = 0; i < state.imus().size(); i++)
            {
                printf("Callback IMUState === IMU State: Temp: %d, Accel: (%.2f, %.2f, %.2f), Angle Rate: (%.2f, %.2f, %.2f), Rpy: (%.2f, %.2f, %.2f)\n",
                state.imus()[i].temperature(),
                state.imus()[i].accelerometer()[0], state.imus()[i].accelerometer()[1], state.imus()[i].accelerometer()[2],
                state.imus()[i].gyroscope()[0], state.imus()[i].gyroscope()[1], state.imus()[i].gyroscope()[2],
                state.imus()[i].rpy()[0], state.imus()[i].rpy()[1], state.imus()[i].rpy()[2]
                );
            }
            
        }  
    });  
      
    while (1)
    {
        usleep(1000);
    }  

    std::cout << "Control sequence completed successfully!" << std::endl;  

    return 0;
}