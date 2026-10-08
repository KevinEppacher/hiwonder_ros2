// Copyright 2026 Kevin Eppacher
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0

#pragma once

#include <vector>

namespace lerobot_teleop
{

class LowPassFilter
{
public:
  LowPassFilter(
    double update_rate,
    double cutoff_frequency,
    bool enabled);

  void reset(
    const std::vector<double> & values);

  [[nodiscard]] std::vector<double> filter(
    const std::vector<double> & input);

private:
  bool enabled_;
  bool initialized_{false};

  double alpha_;

  std::vector<double> state_;
};

}  // namespace lerobot_teleop
