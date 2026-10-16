#include "CollisionGeometry.hh"

#include <cassert>
#include <cmath>

int main()
{
  using scenario02::Intersects;
  using scenario02::OrientedBox;

  const OrientedBox aircraft{
      gz::math::Pose3d(0, 0, 10, 0, 0, 0), {0.3, 0.3, 0.4}};
  assert(Intersects(aircraft, {
      gz::math::Pose3d(0.7, 0, 10, 0, 0, 0), {0.5, 0.2, 0.1}}));
  assert(!Intersects(aircraft, {
      gz::math::Pose3d(0.9, 0, 10, 0, 0, 0), {0.5, 0.2, 0.1}}));
  assert(Intersects(aircraft, {
      gz::math::Pose3d(0.4, 0.4, 10, 0, 0, M_PI_4), {0.5, 0.2, 0.1}}));
  assert(!Intersects(aircraft, {
      gz::math::Pose3d(0, 0, 10.6, 0, 0, 0), {0.5, 0.2, 0.1}}));
  return 0;
}
