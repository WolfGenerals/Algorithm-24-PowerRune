#ifndef TRACKER_HPP
#define TRACKER_HPP
#include <cmath>
#include <vector>

#include "Configuration.hpp"
#include "PowerRune.hpp"

using namespace std::chrono_literals;


class RuneTracker {
public:
    const Configuration& config;


    enum class State {
        /** 跟踪正在击打的能量机关*/
        TRACKING,
        /** 未检测到或无法跟踪能量机关*/
        LOST
    };


    State state = State::LOST;

    PowerRune rune{};

private:
    std::chrono::milliseconds time[3]{};

public:
    void track(const std::vector<FanBlade>& fanBlades, const std::chrono::milliseconds currentTime) {
        // 如果没有检测到叶片，则认为丢失目标
        if (state == State::TRACKING) {
            if (currentTime - time[0] > 500ms) {
                state = State::LOST;
                return;
            }
        }

        time[2] = time[1];
        time[1] = time[0];
        time[0] = currentTime;

        if (state == State::TRACKING) {
            rune.update(fanBlades);
            // 计算转速
            const double speed        = rune.angle * 1000 / (time[0] - time[1]).count();
            const double acceleration = (speed - rune.speed) / ((time[0] - time[2]) / 2).count();
            // 更新转速
            rune.speed        = speed;
            rune.acceleration = acceleration;
        }

        if (state == State::LOST && fanBlades.size() >= 1) {
            rune  = PowerRune{fanBlades[0]};
            state = State::TRACKING;
        }
    }

    explicit RuneTracker(const Configuration& config)
        : config(config) {}
};


#endif //TRACKER_HPP
