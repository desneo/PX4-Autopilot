#pragma once

#include <array>
#include <cmath>
#include <optional>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Box.hh>
#include <sdf/Capsule.hh>
#include <sdf/Cylinder.hh>
#include <sdf/Ellipsoid.hh>
#include <sdf/Geometry.hh>
#include <sdf/Sphere.hh>

namespace scenario02
{
struct OrientedBox
{
  gz::math::Pose3d pose;
  gz::math::Vector3d halfExtents;
};

// 各物理碰撞体的局部外接盒；位姿由 Gazebo 的实际 link / joint 状态给出。
inline std::optional<gz::math::Vector3d> CollisionHalfExtents(
    const sdf::Geometry &_geometry)
{
  if (const auto *box = _geometry.BoxShape();
      _geometry.Type() == sdf::GeometryType::BOX && box)
    return box->Size() * 0.5;
  if (const auto *cylinder = _geometry.CylinderShape();
      _geometry.Type() == sdf::GeometryType::CYLINDER && cylinder)
    return gz::math::Vector3d(
        cylinder->Radius(), cylinder->Radius(), cylinder->Length() * 0.5);
  if (const auto *sphere = _geometry.SphereShape();
      _geometry.Type() == sdf::GeometryType::SPHERE && sphere)
    return gz::math::Vector3d(sphere->Radius(), sphere->Radius(), sphere->Radius());
  if (const auto *capsule = _geometry.CapsuleShape();
      _geometry.Type() == sdf::GeometryType::CAPSULE && capsule)
    return gz::math::Vector3d(capsule->Radius(), capsule->Radius(),
        capsule->Length() * 0.5 + capsule->Radius());
  if (const auto *ellipsoid = _geometry.EllipsoidShape();
      _geometry.Type() == sdf::GeometryType::ELLIPSOID && ellipsoid)
    return ellipsoid->Radii();
  return std::nullopt;
}

inline bool Intersects(const OrientedBox &_a, const OrientedBox &_b)
{
  const std::array<gz::math::Vector3d, 3> axesA{{
      _a.pose.Rot().RotateVector(gz::math::Vector3d::UnitX),
      _a.pose.Rot().RotateVector(gz::math::Vector3d::UnitY),
      _a.pose.Rot().RotateVector(gz::math::Vector3d::UnitZ),
  }};
  const std::array<gz::math::Vector3d, 3> axesB{{
      _b.pose.Rot().RotateVector(gz::math::Vector3d::UnitX),
      _b.pose.Rot().RotateVector(gz::math::Vector3d::UnitY),
      _b.pose.Rot().RotateVector(gz::math::Vector3d::UnitZ),
  }};
  const auto delta = _b.pose.Pos() - _a.pose.Pos();

  const auto separated = [&](const gz::math::Vector3d &_axis)
  {
    if (_axis.SquaredLength() < 1e-12)
      return false;
    double radiusA = 0.0;
    double radiusB = 0.0;
    for (std::size_t index = 0; index < 3; ++index)
    {
      radiusA += std::abs(_axis.Dot(axesA[index])) * _a.halfExtents[index];
      radiusB += std::abs(_axis.Dot(axesB[index])) * _b.halfExtents[index];
    }
    return std::abs(delta.Dot(_axis)) >
        radiusA + radiusB + 1e-9 * _axis.Length();
  };

  // 三维分离轴：两个盒子的六个面法线，加九个棱方向叉积。
  for (const auto &axisA : axesA)
  {
    if (separated(axisA))
      return false;
    for (const auto &axisB : axesB)
      if (separated(axisA.Cross(axisB)))
        return false;
  }
  for (const auto &axisB : axesB)
    if (separated(axisB))
      return false;
  return true;
}
}  // namespace scenario02
