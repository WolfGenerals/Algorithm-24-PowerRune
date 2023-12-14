//
// Created by mojiw on 2023/12/10.
//

#ifndef FILTER_HPP
#define FILTER_HPP
#include "../Interfaces.hpp"


#include <queue>


class Converter {
    std::deque<Direction> centres  = std::deque<Direction>();
    constexpr int         maxCache = 30;

    [[nodiscard]] auto average() const -> Direction;
    [[nodiscard]] auto variance() const -> double;

public:
    Direction gimbal{};

    [[nodiscard]] auto from(const RuneDirection &source) -> RuneDirection;
};

#endif//FILTER_HPP
