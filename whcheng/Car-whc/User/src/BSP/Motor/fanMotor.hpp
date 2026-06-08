#ifndef FAN_MOTOR_HPP
#define FAN_MOTOR_HPP

#include <stdint.h>

#include "drv_pwm.hpp"

namespace BSP::Motor
{
    /**
     * @brief 风扇硬件绑定配置。
     *
     * 由 robot_runtime.hpp 根据 YAML 配置组装，仅包含已构造的 PWM 指针和调速参数。
     */
    struct FanMotorConfig
    {
        /** PWM调速通道指针。 */
        DRV::PWM::IPwm *pwm = nullptr;
        /** 起转占空比（0~1000），用于克服静摩擦力。 */
        uint16_t startup_duty = 200;
        /** 最小运行占空比（0~1000），低于此值自动停转。 */
        uint16_t min_duty = 50;
    };

    /**
     * @brief 12V四线调速鼓风机封装。
     *
     * 通过单路PWM信号控制风扇转速：
     * - 占空比范围0~1000对应0.0%~100.0%
     * - 低于最小占空比时自动停转，避免堵转
     * - 起转时自动以较高占空比启动，克服静摩擦力
     *
     * 四线风扇引脚：VCC(12V)、GND、PWM(调速)、TACH(转速反馈，未使用)
     *
     * 使用方式：
     * - YAML配置方式：通过 FanMotorConfig 构造，由 robot_runtime 自动生成
     * - 手动方式：直接传入 DRV::PWM::IPwm 引用构造
     */
    class FanMotor
    {
    public:
        /**
         * @brief 通过配置结构体构造风扇对象。
         *
         * 适用于 YAML 代码生成流程，由 robot_runtime.hpp 组装 FanMotorConfig 后传入。
         *
         * @param config 风扇硬件绑定配置。
         */
        explicit FanMotor(const FanMotorConfig &config)
            : pwm_(*config.pwm)
            , startup_duty_(config.startup_duty)
            , min_duty_(config.min_duty)
        {
        }

        /**
         * @brief 通过PWM通道直接构造风扇对象。
         *
         * 适用于手动创建场景，使用默认的起转和最小占空比参数。
         *
         * @param pwm PWM输出通道引用。
         */
        explicit FanMotor(DRV::PWM::IPwm &pwm)
            : pwm_(pwm)
        {
        }

        /**
         * @brief 启动风扇。
         *
         * 先以起转占空比启动PWM输出以克服静摩擦力，再恢复目标占空比。
         *
         * @return true 启动成功。
         * @return false PWM无效或HAL启动失败。
         */
        bool Start()
        {
            if (!pwm_.SetDuty(startup_duty_))
            {
                return false;
            }
            if (!pwm_.Start())
            {
                return false;
            }

            // 恢复目标占空比
            pwm_.SetDuty(duty_);
            return true;
        }

        /**
         * @brief 停止风扇。
         *
         * 占空比置零后停止PWM输出。
         *
         * @return true 停止成功。
         * @return false 停止失败。
         */
        bool Stop()
        {
            pwm_.SetDuty(0);
            return pwm_.Stop();
        }

        /**
         * @brief 设置风扇转速占空比。
         *
         * @param duty 占空比，范围0~1000（0.0%~100.0%）。
         *             低于 min_duty_ 时自动钳位为0（停转）。
         * @return true 设置成功。
         */
        bool SetDuty(uint16_t duty)
        {
            if (duty > 1000)
            {
                duty = 1000;
            }

            // 低于最小占空比则停转，防止堵转
            if (duty < min_duty_)
            {
                duty = 0;
            }

            duty_ = duty;
            return pwm_.SetDuty(duty_);
        }

        /**
         * @brief 获取当前占空比。
         *
         * @return uint16_t 当前占空比，范围0~1000。
         */
        uint16_t GetDuty() const
        {
            return duty_;
        }

        /**
         * @brief 以百分比设置风扇转速。
         *
         * @param percent 百分比，范围0.0f~100.0f。
         * @return true 设置成功。
         */
        bool SetPercent(float percent)
        {
            if (percent < 0.0f)
            {
                percent = 0.0f;
            }
            if (percent > 100.0f)
            {
                percent = 100.0f;
            }

            return SetDuty(static_cast<uint16_t>(percent * 10.0f));
        }

    private:
        DRV::PWM::IPwm &pwm_;
        uint16_t startup_duty_ = 200;
        uint16_t min_duty_ = 50;
        uint16_t duty_ = 0;
    };

}

#endif // !FAN_MOTOR_HPP
