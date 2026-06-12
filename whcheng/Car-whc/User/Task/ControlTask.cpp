#include "ControlTask.hpp"
#include "pid.hpp"
#include "robot_messages.hpp"
#include "robot_runtime.hpp"
#include "APP/Fsm/ChassisFSM.hpp"
#include <algorithm>
#include <cmath>

#define TOTLE_HIGH 570.0f - 30.0f - 20.0f // - 13.0f - 40.0f //管高 - 测距高度
#define MASS_KG 0.05f //质量，单位kg

// ==== 激光标定补偿系数（运行 laser_calibration.py 后填入）====
// 误差模型: error(d_meas) = A*d³ + B*d² + C*d + D
// 补偿公式: corrected = d_meas + error(d_meas)
#define LASER_CAL_A  0.00000095f
#define LASER_CAL_B  -0.00089886f
#define LASER_CAL_C  0.19357967f
#define LASER_CAL_D  8.85325744f

inline float CalibrateLaser(float raw_mm)
{
    return raw_mm + (LASER_CAL_A * raw_mm * raw_mm * raw_mm + LASER_CAL_B * raw_mm * raw_mm + LASER_CAL_C * raw_mm + LASER_CAL_D);
}

// ==== Ozone 实时调参 ====
ALG::PID::PidConfig g_dist_pid = RobotConfig::kPidHighDistancePid;
float g_ude_gain    = 0.0f;     // UDE 输出 → 占空比 转换系数
float g_launch_duty = 200.0f;   // 发射占空比（吹到顶）
float g_hover_bias  = 190.0f;   // 悬停偏置（PID 围绕此值修正）
int   g_state       = 0;        // 0=空闲, 1=发射, 2=PID控制
bool  g_first_key3  = true;     // Key2 发射后首次 Key3 设 target=100

// Ozone 观测 PID 分量
float g_pid_pout = 0.0f;
float g_pid_iout = 0.0f;
float g_pid_dout = 0.0f;
float g_pid_err  = 0.0f;

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

    // Key1: 目标 -50
    if (btn_now[0] && !btn_prev[0])
        high_target.target_distance_mm -= 50;

    // Key2: 发射
    if (btn_now[1] && !btn_prev[1])
    {
        g_state = 1;
        g_first_key3 = true;
    }

    // Key3: 首次=100，之后+50，同时激活 PID
    if (btn_now[2] && !btn_prev[2])
    {
        if (g_first_key3)
        {
            high_target.target_distance_mm = 100;
            g_first_key3 = false;
        }
        else
        {
            high_target.target_distance_mm += 50;
        }
        g_state = 2;
    }

    high_target.target_distance_mm = std::clamp(high_target.target_distance_mm, int16_t{0}, int16_t{400});

    btn_prev[0] = btn_now[0];
    btn_prev[1] = btn_now[1];
    btn_prev[2] = btn_now[2];

    PublishHighTargetData(high_target);
}
float whatch_acc;
void Control()
{
    auto &dist_pid = HighDistancePidPid();

    if (g_state == 0)
    {
        // 空闲：输出为0
        motor_output.high_out = 0;
        PublishMotorOutData(motor_output);
        return;
    }

    if (g_state == 1)
    {
        // 发射：固定占空比，PID 跟踪但不输出
        dist_pid.SetConfig(g_dist_pid);
        dist_pid.Update(high_target.target_distance_mm, feedback_data.feedback_distance_mm);
        motor_output.high_out = std::clamp(g_launch_duty, 0.0f, 300.0f);
        PublishMotorOutData(motor_output);
        return;
    }

    // PID 控制模式
    dist_pid.SetConfig(g_dist_pid);
    dist_pid.Update(high_target.target_distance_mm, feedback_data.feedback_distance_mm);

    float pid_output = dist_pid.GetOutput();

    // Ozone 观测 PID 各项
    g_pid_err  = dist_pid.GetError();
    g_pid_pout = g_dist_pid.kp * g_pid_err;
    g_pid_iout = g_dist_pid.ki * dist_pid.GetIntegral();
    g_pid_dout = -g_dist_pid.kd * dist_pid.GetTdDerivative();

    float totle_output = g_hover_bias + pid_output;

    motor_output.high_out = std::clamp(totle_output, 0.0f, 300.0f);
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
