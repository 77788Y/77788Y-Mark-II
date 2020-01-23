#include "subsystems/tilter.hpp"
#include <memory>

// constructor
Tilter::Tilter(std::unique_ptr<Transmission> transmission):
  m_transmission(std::move(transmission)), velmath(VelMath(VelMathFactory::create(360, 5_ms))) {}

// move voltage
void Tilter::move_voltage(int val) {
  m_transmission->m_state = Transmission::State::LOCKED_PASSTHROUGH;
  m_transmission->m_desired_tilter_voltage = val;
}

// extend/retract tray
void Tilter::extend_passive() {
  m_transmission->m_state = Transmission::State::EXTENDING;
}
void Tilter::retract_passive() {
  m_transmission->m_state = Transmission::State::RETRACTING;
}

// get pose
QAngle Tilter::get_pose() {
  return m_pose;
}
QAngularSpeed Tilter::get_velocity() {
  return velmath.getVelocity();
}
QAngularAcceleration Tilter::get_acceleration() {
  return velmath.getAccel();
}

// tare pose
void Tilter::tare_pose(QAngle new_pose) {
  m_reference_pose = new_pose - m_absolute_pose;
  update_pose();
}

// update pose
void Tilter::update_pose() {

  // calculate new absolute pose
  m_absolute_pose = (
    m_transmission->m_ime_left_shared->get() - m_transmission->m_ime_left_direct->get() +
    m_transmission->m_ime_right_shared->get() - m_transmission->m_ime_right_direct->get()
  ) * .5_deg;

  // update velmath
  velmath.step(m_absolute_pose.convert(degree));
  
  // update pose
  m_pose = m_absolute_pose + m_reference_pose;
}