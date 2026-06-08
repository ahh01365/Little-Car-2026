#include "CommunicationTask.hpp"

#include "cmsis_os.h"
#include "robot_messages.hpp"
#include "robot_runtime.hpp"
#include <string.h>

namespace
{
    uint8_t send_str2[6 * sizeof(float) + 4] = {};

#define SUB_MSG(T, var)                     \
    RobotMessages::T var{};                  \
    void On##T(const RobotMessages::T &m) { var = m; }

    SUB_MSG(HighTargetData, target_data)
#undef SUB_MSG

    void InitMessageSubscribe()
    {
        RobotMessages::SubscribeHighTargetData(OnHighTargetData);
    }
}

void vofa_send(float x1, float x2, float x3, float x4, float x5, float x6)
{
    const float data[6] = {x1, x2, x3, x4, x5, x6};
    memcpy(send_str2, data, sizeof(data));

    send_str2[sizeof(data) + 0] = 0x00;
    send_str2[sizeof(data) + 1] = 0x00;
    send_str2[sizeof(data) + 2] = 0x80;
    send_str2[sizeof(data) + 3] = 0x7F;

    // DRV::UART::UartManager::Instance().Send(
    //     DRV::UART::UartId::Debug,
    //     send_str2,
    //     sizeof(send_str2)
    // );
}

void Laser_OnUartRx(DRV::UART::UartId id, const DRV::UART::UartData &data)
{
    auto &laser = RobotRuntime::Laser();
    laser.OnUartRx(data.buffer, data.size);
    if (laser.IsValid())
    {
        RobotMessages::LaserData msg{};
        msg.distance_mm = static_cast<int16_t>(laser.GetDistance());
        PublishLaserData(msg);
    }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    RobotRuntime::OnUartRxEvent(huart, Size);
}

extern "C" void CommTask(void const *argument)
{
    RobotRuntime::InitUarts();
    InitMessageSubscribe();

    for (;;)
    {
        osDelay(33);
    }
}
