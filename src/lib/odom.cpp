#include "lib/odom.hpp"

// constructor
Odom::Odom(
  std::unique_ptr<ContinuousRotarySensor> enc_left,
  std::unique_ptr<ContinuousRotarySensor> enc_right,
  std::unique_ptr<ContinuousRotarySensor> enc_side,
  std::shared_ptr<ContinuousRotarySensor> enc_left_backup,
  std::shared_ptr<ContinuousRotarySensor> enc_right_backup ,
  std::unique_ptr<pros::Imu> imu,
  QLength track_width, QLength secondary_track_width, QLength side_dist, QLength wheel_radius
):
  m_enc_left(std::move(enc_left)),
  m_enc_right(std::move(enc_right)),
  m_enc_side(std::move(enc_side)),
  m_enc_left_backup(enc_left_backup),
  m_enc_right_backup(enc_right_backup),
  m_imu(std::move(imu)),
  m_track_width(track_width),
  m_secondary_track_width(secondary_track_width),
  m_side_dist(side_dist),
  m_wheel_radius(wheel_radius)
{}

// update
void Odom::update() {
  // TODO: odom
}

// get pose
Odom::ChassisPose* Odom::get_pose() {
  return m_pose.get();
}

// get rate of change
Odom::ChassisDeriv* Odom::get_speed() {
  return m_deriv.get();
}
