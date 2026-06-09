#include "HardwareTask.hpp"

#include <algorithm>
#include <stdint.h>

#include "robot_messages.hpp"
#include "robot_runtime.hpp"
#include "cmsis_os.h"

using namespace RobotMessages;
using namespace RobotRuntime;

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

void motor_control_logic()
{
    auto &fan = BlowerFan();
    fan.SetDuty(static_cast<uint16_t>(motor_output.high_out));
}

extern "C" void HardwareTask(void const *argument)
{
    BlowerFan().Start();
    InitMessageSubscribe();

    for (;;) 
    {
        motor_control_logic();
        osDelay(1);
    }
}
