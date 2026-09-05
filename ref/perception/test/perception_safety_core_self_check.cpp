#include <cassert>
#include <cmath>
#include <vector>

#include "perception/perception_safety_core.hpp"

int main()
{
  using namespace perception::safety;

  // Obstacle recenter gate: object kecil/noise tidak boleh memblokir.
  ObstacleGateConfig config;
  config.minimum_obstacle_width_m = 0.50;
  const std::vector<ObstacleMetric> small{{1, 1.0, 0.0, 0.20}};
  assert(!obstacleBlocksRecenter(0.0, small, config).blocked);

  // Object valid yang memotong swept lateral corridor harus memblokir recenter.
  const std::vector<ObstacleMetric> valid{{2, 1.0, 0.0, 0.50}};
  assert(obstacleBlocksRecenter(0.0, valid, config).blocked);

  // Object di luar forward gate tidak boleh memblokir recenter.
  const std::vector<ObstacleMetric> far{{3, 4.0, 0.0, 0.80}};
  assert(!obstacleBlocksRecenter(0.0, far, config).blocked);

  LaneThresholds thresholds;

  // Di area aman, Nav2 tetap NORMAL.
  const auto normal = classifyLaneState(true, 1.5, 1.5, 0.0, NORMAL, thresholds);
  assert(normal.state == NORMAL);
  assert(!normal.critical);

  // Mepet kanan berarti harus geser kiri; 0.35 m juga masuk critical <= 0.40 m.
  const auto near_right = classifyLaneState(true, 1.6, 0.35, 0.30, NORMAL, thresholds);
  assert(near_right.state == RECENTER_LEFT);
  assert(near_right.critical);

  // Mepet kiri berarti harus geser kanan.
  const auto near_left = classifyLaneState(true, 0.35, 1.6, -0.30, NORMAL, thresholds);
  assert(near_left.state == RECENTER_RIGHT);
  assert(near_left.critical);

  // Geometry invalid tetap ditandai lane lost, bukan menghasilkan steering liar.
  const auto lost = classifyLaneState(false, 0.0, 0.0, 0.0, NORMAL, thresholds);
  assert(lost.state == LANE_LOST);

  // Mixer harus mempertahankan arah maju, membatasi speed recenter, dan finite.
  MixerConfig mixer;
  const auto cmd = mixRecenterCommand(0.30, 0.0, 0.50, 0.0, false, mixer);
  assert(cmd.linear_x > 0.0 && cmd.linear_x <= mixer.recenter_speed_mps + 1.0e-9);
  assert(std::isfinite(cmd.angular_z));
  assert(cmd.angular_z > 0.0);

  return 0;
}
