#ifndef POWERRUNE_HPP
#define POWERRUNE_HPP
#include <optional>


/** \brief R标 */
struct RIcon {
    cv::Point2d position;
    double      range;
};


/** \brief 扇叶状态 */
enum class RuneState {
    /** \brief 未激活，发光 */
    INACTIVE,
    /** \brief 已激活，发光 */
    ACTIVE,
    /** \brief 不可激活，不发光 */
    DISABLED
};


/** \brief 扇叶 */
struct FanBlade {
    /** \brief 偏移量  */
    cv::Point2d offset;
    /** \brief 状态 */
    RuneState state = RuneState::DISABLED;

    [[nodiscard]]
    double legth() const { return sqrt(offset.x * offset.x + offset.y * offset.y); }

    [[nodiscard]]
    FanBlade rotate(double radian) const {
        return {
                    {
                        offset.x * cos(radian) - offset.y * sin(radian),
                        offset.x * sin(radian) + offset.y * cos(radian)
                    },
                    state
                };
    }
};


// 定义一个能量符文的结构体，其中包含图标中心点信息
/** \brief 能量机关 */
struct PowerRune {
    FanBlade fanBlades[5]; /** \brief 扇叶 */

    double length = 0.0;//米
    double angle = 0.0;//弧度
    double speed = 0.0;//弧度/秒
    double acceleration = 0.0;//弧度/秒^2

    /** 更新叶片长度 */
    void updateLength(const double length) {
        for (auto& fanBlade: fanBlades)
            fanBlade.offset = fanBlade.offset * length / fanBlade.legth();
    }

    /** 更新叶片角度 */
    void updateAngle(const double angle) {
        for (auto& fanBlade: fanBlades)
            fanBlade = fanBlade.rotate(angle);
    }

    /** 寻找距离最近的扇叶 */
    FanBlade* march(const FanBlade& blade) {
        FanBlade* marched = nullptr;
        auto      min     = DBL_MAX;
        for (auto& fanBlade: fanBlades) {
            const double distance = (fanBlade.offset.x - blade.offset.x) * (fanBlade.offset.x - blade.offset.x) + (fanBlade.offset.y - blade
               .
                offset.y) * (
                fanBlade.offset.y - blade.offset.y);
            if (min <= distance)
                continue;
            min     = distance;
            marched = &fanBlade;
        }
        return marched;
    }


    void update(const std::vector<FanBlade>& fanBlades) {
        //如果没有亮起的扇叶，则跳过更新
        if (fanBlades.empty())
            return;

        angle  = 0;
        length = 0;
        for (auto& fanBlade: fanBlades) {
            //寻找匹配的扇叶
            const FanBlade* marched = march(fanBlade);
            // 计算平均叶片长度
            length += fanBlade.legth() / static_cast<double>(fanBlades.size());
            // 计算平均偏移角度
            angle += asin(marched->offset.cross(fanBlade.offset)/marched->legth()/fanBlade.legth()) / static_cast<double>(fanBlades.size());
        }
        // 更新叶片长度角度
        updateLength(length);
        updateAngle(angle);
        // 更新叶片状态
        //如果只有一个亮起的叶片且亮起的叶片改变，则认为能量机关激活状态已经重置
        if (fanBlades.size() == 1) {
            FanBlade* fanBlade = march(fanBlades[0]);
            if (fanBlade->state == RuneState::DISABLED)
                return;
            for (auto& [offset, state]: this->fanBlades)
                state = RuneState::DISABLED;
            fanBlade->state = RuneState::INACTIVE;
            return;
        }
        for (auto& fanBlade: fanBlades) {
            //寻找匹配的扇叶
            FanBlade* marched = march(fanBlade);
            // 如果从不可激活变为亮起状态，则认为之前的可激活状态的叶片已经激活，新亮起的为可激活状态
            if (marched->state == RuneState::DISABLED && fanBlade.state == RuneState::INACTIVE) {
                for (auto& [offset, state]: this->fanBlades)
                    if (state == RuneState::INACTIVE)
                        state = RuneState::ACTIVE;
                marched->state = RuneState::INACTIVE;
            }
        }
    }

    auto targe() -> std::optional<FanBlade> {
        for (auto& fanBlade: fanBlades)
            if (fanBlade.state == RuneState::INACTIVE)
                return fanBlade;
        return std::nullopt;
    }

    PowerRune() = default;

    explicit PowerRune(const FanBlade& blade) {
        fanBlades[0]       = blade;
        fanBlades[0].state = RuneState::INACTIVE;
        // 其余叶片通过旋转第一片72度获得
        for (int i = 1; i < 5; i++) {
            fanBlades[i].offset = blade.rotate(i * 72 * M_PI / 180).offset;
            fanBlades[i].state  = RuneState::DISABLED;
        }
    };
};


#endif //POWERRUNE_HPP
