#ifndef CHASSISFSM_HPP
#define CHASSISFSM_HPP

#include "APP/FsmTools/fsmtools.hpp"

namespace APP::Fsm
{
    class ChassisFSM
    {
    public:
        enum class State : uint8_t { FULL, SLOW, TURN, STOP, COUNT };
        static constexpr uint8_t kStateCount = static_cast<uint8_t>(State::COUNT);

        struct Config
        {
            uint32_t lost_timeout;
        };

        ChassisFSM() = default;

        explicit ChassisFSM(const Config &cfg)
        {
            Init(cfg);
        }

        void Init(const Config &cfg)
        {
            lost_timeout_ = cfg.lost_timeout;
            static const char *kNames[] = {"FULL", "SLOW", "TURN", "STOP"};
            fsm_.Init(State::FULL, kNames);
        }

        // 状态切换条件：
        // valid==0  → 开始计数，超时 → STOP
        // valid==1 && vision_state==0  → FULL
        // valid==1 && vision_state==1  → SLOW
        // valid==1 && vision_state==2,3 → TURN
        // valid==1 && vision_state==9  → STOP
        State Update(int32_t vision_state, int32_t valid)
        {
            if (valid == 0)
            {
                lost_count_++;
                if (lost_count_ > lost_timeout_)
                {
                    fsm_.TransitionTo(State::STOP);
                }
            }
            else
            {
                lost_count_ = 0;
                switch (vision_state)
                {
                    case 0:  fsm_.TransitionTo(State::FULL); break;
                    case 1:  fsm_.TransitionTo(State::SLOW); break;
                    case 2:  
                    case 3:  fsm_.TransitionTo(State::TURN); break;
                    default: fsm_.TransitionTo(State::STOP); break;
                }
            }

            fsm_.Tick();
            return fsm_.GetState();
        }

        State GetState() const { return fsm_.GetState(); }

    private:
        uint32_t lost_timeout_ = 100;
        uint32_t lost_count_ = 0;
        FsmTools::FsmBase<State, kStateCount> fsm_{};
    };
}

#endif // !CHASSISFSM_HPP
