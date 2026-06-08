#include "ControlTask.hpp"
#include "robot_messages.hpp"
#include "robot_runtime.hpp"
#include "APP/Fsm/ChassisFSM.hpp"

#include <cmath>

using namespace RobotRuntime;
using namespace RobotMessages;
using State = APP::Fsm::ChassisFSM::State;

namespace
{
#define SUB_MSG(T, var)                     \
    RobotMessages::T var{};                  \
    void On##T(const RobotMessages::T &m) { var = m; }

    SUB_MSG(HighTargetData, high_target)
    SUB_MSG(LaserData, laser_data)
#undef SUB_MSG

    RobotMessages::MotorOutData motor_output{};

    void InitMessageSubscribe()
    {
        RobotMessages::SubscribeHighTargetData(OnHighTargetData);
        RobotMessages::SubscribeLaserData(OnLaserData);
    }
}


void SetTarget()
{

}

void Control()
{

    
    PublishMotorOutData(motor_output);
}

extern "C" void ControTask(void const *argument)
{
    InitMessageSubscribe();
    
    for (;;)
    {
        SetTarget();
        Control();
        osDelay(1);
    }
}
