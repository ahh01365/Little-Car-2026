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
    // // === 鼓风机测试模式 ===
    // uint32_t compare = (39u * static_cast<uint32_t>(motor_output.high_out)) / 1000u;
    // a = compare;
    // __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, compare);

    // === 涵道ESC模式 ===
    // 0~1000 → ESC 脉宽 1100~1940μs，带软启动爬升
    uint32_t target = 1100u + (static_cast<uint32_t>(motor_output.high_out) * 840u) / 1000u;

    static uint32_t current = 1100u;
    if (current < target)  current += 1;   // 每 1ms 爬 1μs
    if (current > target)  current = target;

    
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
