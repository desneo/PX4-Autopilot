#pragma once

#include <array>
#include <cmath>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>

namespace scenario02
{
struct OrientedBox
{
  gz::math::Pose3d pose;
  gz::math::Vector3d halfExtents;
};

inline bool Intersects(const OrientedBox &_a, const OrientedBox &_b)
{
  if (std::abs(_a.pose.Pos().Z() - _b.pose.Pos().Z()) >
      _a.halfExtents.Z() + _b.halfExtents.Z())
    return false;

  const double yawA = _a.pose.Rot().Yaw();
  const double yawB = _b.pose.Rot().Yaw();
  const std::array<gz::math::Vector2d, 2> axesA{{
      {std::cos(yawA), std::sin(yawA)},
      {-std::sin(yawA), std::cos(yawA)},
  }};
  const std::array<gz::math::Vector2d, 2> axesB{{
      {std::cos(yawB), std::sin(yawB)},
      {-std::sin(yawB), std::cos(yawB)},
  }};
  const gz::math::Vector2d delta(
      _b.pose.Pos().X() - _a.pose.Pos().X(),
      _b.pose.Pos().Y() - _a.pose.Pos().Y());

  const auto separated = [&](const gz::math::Vector2d &_axis)
  {
    const double radiusA =
        std::abs(_axis.Dot(axesA[0])) * _a.halfExtents.X() +
        std::abs(_axis.Dot(axesA[1])) * _a.halfExtents.Y();
    const double radiusB =
        std::abs(_axis.Dot(axesB[0])) * _b.halfExtents.X() +
        std::abs(_axis.Dot(axesB[1])) * _b.halfExtents.Y();
    return std::abs(delta.Dot(_axis)) > radiusA + radiusB;
  };

  return !separated(axesA[0]) && !separated(axesA[1]) &&
      !separated(axesB[0]) && !separated(axesB[1]);
}
}  // namespace scenario02
