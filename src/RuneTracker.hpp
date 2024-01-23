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

private:
    enum class State {
        /** 跟踪正在击打的能量机关*/
        TRACKING,
        /** 未检测到或无法跟踪能量机关*/
        LOST
    };


    State state = State::LOST;

    PowerRune rune{};
    std::chrono::milliseconds    lastUpdateTime{};

public:
    auto track(const std::vector<FanBlade>& fanBlades, const std::chrono::milliseconds currentTime) -> std::optional<PowerRune> {
        // 如果没有检测到叶片，则认为丢失目标
        if (fanBlades.empty() && state == State::TRACKING) {
            state = State::LOST;
            return std::nullopt;
        }
        // 间隔时间过长，则认为信息已经过期
        if (currentTime - lastUpdateTime > 500ms)
            state = State::LOST;

        if (state == State::TRACKING)
            rune.update(fanBlades, currentTime - lastUpdateTime);

        if (state == State::LOST && fanBlades.size() == 1) {
            rune  = PowerRune{fanBlades[0]};
            state = State::TRACKING;
        }


        //更新时间
        lastUpdateTime = currentTime;

        if (state == State::TRACKING)
            return rune;
        return std::nullopt;
    }

    explicit RuneTracker(const Configuration& config)
        : config(config) {}
};


#endif //TRACKER_HPP
