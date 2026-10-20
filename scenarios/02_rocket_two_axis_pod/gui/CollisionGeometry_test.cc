#include "CollisionGeometry.hh"

#include <cmath>
#include <stdexcept>
#include <string>

// Release 构建也必须执行检查，不能依赖会被 NDEBUG 移除的 assert。
void CheckAt(bool _condition, int _line)
{
  if (!_condition)
    throw std::runtime_error("collision geometry check failed at line " + std::to_string(_line));
}
#define Check(condition) CheckAt((condition), __LINE__)

int main()
{
  using scenario02::Intersects;
  using scenario02::OrientedBox;

  const OrientedBox aircraft{
      gz::math::Pose3d(0, 0, 10, 0, 0, 0), {0.3, 0.3, 0.4}};
  Check(Intersects(aircraft, {
      gz::math::Pose3d(0.7, 0, 10, 0, 0, 0), {0.5, 0.2, 0.1}}));
  Check(!Intersects(aircraft, {
      gz::math::Pose3d(0.9, 0, 10, 0, 0, 0), {0.5, 0.2, 0.1}}));
  Check(Intersects(aircraft, {
      gz::math::Pose3d(0.4, 0.4, 10, 0, 0, M_PI_4), {0.5, 0.2, 0.1}}));
  Check(!Intersects(aircraft, {
      gz::math::Pose3d(0, 0, 10.6, 0, 0, 0), {0.5, 0.2, 0.1}}));
  const OrientedBox upright{gz::math::Pose3d::Zero, {0.045, 0.045, 0.2495}};
  const OrientedBox nearTip{gz::math::Pose3d(0.25, 0, 0, 0, 0, 0), {0.02, 0.02, 0.02}};
  Check(!Intersects(upright, nearTip));
  auto pitched = upright;
  pitched.pose.Rot() = gz::math::Quaterniond(0, M_PI_2, 0);
  Check(Intersects(pitched, nearTip));
  Check(Intersects(nearTip, pitched));
  auto rolled = upright;
  rolled.pose.Rot() = gz::math::Quaterniond(M_PI_2, 0, 0);
  const OrientedBox nearRollTip{
      gz::math::Pose3d(0, 0.25, 0, 0, 0, 0), {0.02, 0.02, 0.02}};
  Check(Intersects(rolled, nearRollTip));
  Check(!Intersects(pitched, {
      gz::math::Pose3d(0, 0, 0.1, 0, 0, 0), {0.02, 0.02, 0.02}}));

  // 坐标整体旋转和平移不应改变相交结果。
  const gz::math::Pose3d transform(20, -30, 100, 0.6, -0.8, 0.4);
  Check(Intersects({transform * pitched.pose, pitched.halfExtents},
      {transform * nearTip.pose, nearTip.halfExtents}));
  Check(!Intersects({transform * upright.pose, upright.halfExtents},
      {transform * nearTip.pose, nearTip.halfExtents}));

  sdf::Geometry geometry;
  sdf::Cylinder cylinder;
  cylinder.SetRadius(0.045);
  cylinder.SetLength(0.499);
  geometry.SetCylinderShape(cylinder);
  geometry.SetType(sdf::GeometryType::CYLINDER);
  const auto half = scenario02::CollisionHalfExtents(geometry);
  Check(half.has_value());
  Check(half->Equal({0.045, 0.045, 0.2495}, 1e-12));
  sdf::Box box;
  box.SetSize({0.2, 0.4, 0.6});
  geometry.SetBoxShape(box);
  geometry.SetType(sdf::GeometryType::BOX);
  Check(scenario02::CollisionHalfExtents(geometry)->Equal({0.1, 0.2, 0.3}, 1e-12));
  sdf::Sphere sphere;
  sphere.SetRadius(0.024);
  geometry.SetSphereShape(sphere);
  geometry.SetType(sdf::GeometryType::SPHERE);
  Check(scenario02::CollisionHalfExtents(geometry)->Equal({0.024, 0.024, 0.024}, 1e-12));
  Check(!scenario02::CollisionHalfExtents(sdf::Geometry{}).has_value());
  return 0;
}
