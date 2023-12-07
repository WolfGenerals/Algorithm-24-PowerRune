//
// Created by mojiw on 2023/12/6.
//

#ifndef INTERFACES_HPP
#define INTERFACES_HPP

struct  Direction {
  const double pitch;
  const double yaw;
};

struct Arm {
  const Direction target;
  const Direction centre;
};
#endif // INTERFACES_HPP
