#include "HardwareTask.hpp"

#include <algorithm>
#include <stdint.h>

#include "robot_messages.hpp"
#include "cmsis_os.h"
#include "tim.h"

using namespace RobotMessages;

namespace
{
    RobotMessages::MotorOutData motor_output{};

    void OnMotorOutput(const RobotMessages::MotorOutData &msg)
    {
        motor_output = msg;
    }

    void InitMessageSubscribe()
    {
        RobotMessages::SubscribeMotorOutData(OnMotorOutput);
    }
}
int a = 0;

void motor_control_logic()
{
    a = std::clamp(a, 0, 300);
    // 目标脉宽
    uint32_t target = 1100u + (static_cast<uint32_t>(a) * 840u) / 1000u;//1100u + (static_cast<uint32_t>(motor_output.high_out) * 840u) / 1000u;

    // 当前脉宽（静态变量，缓慢爬升）
    static uint32_t current = 1100u;
    if (current < target)  current += 1;   // 每 1ms 爬 1μs
    if (current > target)  current = target; // 降低则立刻响应

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, current);
}

extern "C" void HardwareTask(void const *argument)
{
    InitMessageSubscribe();

    // 等电调解锁音（PWM 已在 MX_TIM1_Init 中以 1100μs 启动）
    osDelay(3000);

    for (;;)
    {
        motor_control_logic();
        osDelay(1);
    }
}
