#include "subsystems/intake.hpp"

// constructor
Intake::Intake(int8_t port_l, int8_t port_r):
  m_motor_left (std::make_unique<Motor>(port_l, false, Motor::gearset::green, Motor::encoderUnits::degrees)),
  m_motor_right(std::make_unique<Motor>(port_r, true, Motor::gearset::green, Motor::encoderUnits::degrees)),
  velmath_left(VelMathFactory::create(360, 5_ms)),
  velmath_right(VelMathFactory::create(360, 5_ms))
{}

// move voltage
void Intake::move_voltage(int val) {
  m_motor_left ->setBrakeMode(Motor::brakeMode::coast);
  m_motor_right->setBrakeMode(Motor::brakeMode::coast);
  m_motor_left ->moveVoltage(val);
  m_motor_right->moveVoltage(val);
}

// lock motors
void Intake::lock() {
  m_motor_left ->setBrakeMode(Motor::brakeMode::hold);
  m_motor_right->setBrakeMode(Motor::brakeMode::hold);
  m_motor_left ->moveVelocity(0);
  m_motor_right->moveVelocity(0);
}

// get angle
std::tuple<QAngle, QAngle, QAngle> Intake::get_angle() {
  return std::tuple<QAngle, QAngle, QAngle>(
    m_pose_left, 
    m_pose_right, 
    (m_pose_left + m_pose_right) * .5
  );
}

// get velocity
std::tuple<QAngularSpeed, QAngularSpeed, QAngularSpeed> Intake::get_velocity() {
  return std::tuple<QAngularSpeed, QAngularSpeed, QAngularSpeed>(
    velmath_left.getVelocity(), 
    velmath_right.getVelocity(), 
    (velmath_left.getVelocity() + velmath_right.getVelocity()) * .5
  );
}

// get acceleration
std::tuple<QAngularAcceleration, QAngularAcceleration, QAngularAcceleration> Intake::get_acceleration() {
  return std::tuple<QAngularAcceleration, QAngularAcceleration, QAngularAcceleration>(
    velmath_left.getAccel(), 
    velmath_right.getAccel(), 
    (velmath_left.getAccel() + velmath_right.getAccel()) * .5
  );
}

// tare angles
void Intake::tare_angles(QAngle left, QAngle right) {
  m_reference_pose_left =  left  - m_absolute_pose_left;
  m_reference_pose_right = right - m_absolute_pose_right;
  update_angles();
}

// update angles
void Intake::update_angles() {
  m_absolute_pose_left = m_motor_left->getPosition() * 1_deg;
  m_absolute_pose_right = m_motor_right->getPosition() * 1_deg;
  velmath_left.step(m_absolute_pose_left.convert(degree));
  velmath_right.step(m_absolute_pose_right.convert(degree));
  m_pose_left = m_absolute_pose_left + m_reference_pose_left;
  m_pose_right = m_absolute_pose_right + m_reference_pose_right;
}