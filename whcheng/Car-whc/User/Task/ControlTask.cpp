#include "ControlTask.hpp"
#include "pid.hpp"
#include "robot_messages.hpp"
#include "robot_runtime.hpp"
#include "APP/Fsm/ChassisFSM.hpp"
#include <algorithm>
#include <cmath>

#define TOTLE_HIGH 500.0f - 23.0f - 40.0f //0.5m
#define MASS_KG 0.05f //质量，单位kg

// ==== 激光标定补偿系数（运行 laser_calibration.py 后填入）====
// 误差模型: error(d_meas) = A*d² + B*d + C
// 补偿公式: corrected = d_meas + error(d_meas)
#define LASER_CAL_A  0.0f
#define LASER_CAL_B  0.0f
#define LASER_CAL_C  0.0f

inline float CalibrateLaser(float raw_mm)
{
    return raw_mm + (LASER_CAL_A * raw_mm * raw_mm + LASER_CAL_B * raw_mm + LASER_CAL_C);
}

using namespace RobotRuntime;
using namespace RobotMessages;

namespace
{
#define SUB_MSG(T, var)                     \
    RobotMessages::T var{};                  \
    void On##T(const RobotMessages::T &m) { var = m; }

    SUB_MSG(LaserData, laser_data)
#undef SUB_MSG

    RobotMessages::MotorOutData motor_output{};
    RobotMessages::HighFeedbackData feedback_data{};
    RobotMessages::HighTargetData high_target{};

    // 按键边沿检测（active_low: 按下=低, 松开=高）
    bool btn_prev[3] = {false, false, false};
    // 上次反馈的速度，用于计算加速度
    float prev_feedback_velocity_mmps = 0.0f;

    void InitMessageSubscribe()
    {
        RobotMessages::SubscribeLaserData(OnLaserData);
    }
}

void SetFeedback()
{
    float corrected_distance = CalibrateLaser(laser_data.distance_mm);
    float filtered_distance = HighDistanceTdTdFilter().Filter(corrected_distance);
    feedback_data.feedback_distance_mm = TOTLE_HIGH - filtered_distance;
    feedback_data.feedback_velocity_mmps = HighDistanceTdTdFilter().GetDifferentialValue();
    PublishHighFeedbackData(feedback_data);
}

void SetTarget()
{
    auto &btns = Buttons();

    bool btn_now[3] = 
    {
        btns[0].IsPressed(),
        btns[1].IsPressed(),
        btns[2].IsPressed(),
    };

    // Key1: 下降沿（刚按下）→ 目标 -50
    if (btn_now[0] && !btn_prev[0])
        high_target.target_distance_mm -= 50;

    // Key2: 下降沿 → 目标归零
    if (btn_now[1] && !btn_prev[1])
        high_target.target_distance_mm = 0;

    // Key3: 下降沿 → 目标 +50
    if (btn_now[2] && !btn_prev[2])
        high_target.target_distance_mm += 50;

    high_target.target_distance_mm = std::clamp(high_target.target_distance_mm, int16_t{0}, int16_t{400});

    btn_prev[0] = btn_now[0];
    btn_prev[1] = btn_now[1];
    btn_prev[2] = btn_now[2];

    PublishHighTargetData(high_target);
}

void Control()
{
    auto &dist_pid  = HighDistancePidPid();
    auto &vel_pid   = HighVelocityPidPid();
    auto &gravity = GravityForwardFeedforward(); 

    dist_pid.Update(high_target.target_distance_mm, feedback_data.feedback_distance_mm);
    vel_pid.Update(dist_pid.GetOutput(), feedback_data.feedback_velocity_mmps);

    float pid_output = vel_pid.GetOutput();
    float gravity_output = gravity.Update(0.0f);

    HighVelocityTdTdFilter().Filter(prev_feedback_velocity_mmps);
    float high_acc = HighVelocityTdTdFilter().GetDifferentialValue();

    float d = high_acc - (1/MASS_KG * pid_output);
    float ude_output = UdeFilterTdTdFilter().Filter(d);

    float totle_output = pid_output + gravity_output - ude_output;

    motor_output.high_out = std::clamp(totle_output, 0.0f, 1000.0f);
    PublishMotorOutData(motor_output);

    prev_feedback_velocity_mmps = feedback_data.feedback_velocity_mmps;
}


extern "C" void ControTask(void const *argument)
{
    InitMessageSubscribe();

    for (;;)
    {
        SetFeedback();
        SetTarget();
        Control();
        osDelay(1);
    }
}
