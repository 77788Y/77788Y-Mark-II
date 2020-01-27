#include "subsystems/lift.hpp"

// constructor
Lift::Lift(int8_t port_l, int8_t port_r):
  m_motor_left (std::make_unique<Motor>(port_l, false, Motor::gearset::green, Motor::encoderUnits::degrees)),
  m_motor_right(std::make_unique<Motor>(port_r, true, Motor::gearset::green, Motor::encoderUnits::degrees)),
  velmath_left(VelMathFactory::create(360, 5_ms)),
  velmath_right(VelMathFactory::create(360, 5_ms))
{}

// move voltage
void Lift::move_voltage(int val) {
  m_motor_left ->setBrakeMode(Motor::brakeMode::coast);
  m_motor_right->setBrakeMode(Motor::brakeMode::coast);
  m_motor_left ->moveVoltage(val + (m_motor_right->getPosition() - m_motor_left->getPosition()) * 20);
  m_motor_right->moveVoltage(val - (m_motor_right->getPosition() - m_motor_left->getPosition()) * 20);
}

// lock motors
void Lift::lock() {
  m_motor_left ->setBrakeMode(Motor::brakeMode::hold);
  m_motor_right->setBrakeMode(Motor::brakeMode::hold);
  m_motor_left ->moveVelocity(0);
  m_motor_right->moveVelocity(0);
}

// get angle
std::tuple<QAngle, QAngle, QAngle> Lift::get_angle() {
  return std::tuple<QAngle, QAngle, QAngle>(
    m_pose_left, 
    m_pose_right, 
    (m_pose_left + m_pose_right) * .5
  );
}

// get velocity
std::tuple<QAngularSpeed, QAngularSpeed, QAngularSpeed> Lift::get_velocity() {
  return std::tuple<QAngularSpeed, QAngularSpeed, QAngularSpeed>(
    velmath_left.getVelocity(), 
    velmath_right.getVelocity(), 
    (velmath_left.getVelocity() + velmath_right.getVelocity()) * .5
  );
}

// get acceleration
std::tuple<QAngularAcceleration, QAngularAcceleration, QAngularAcceleration> Lift::get_acceleration() {
  return std::tuple<QAngularAcceleration, QAngularAcceleration, QAngularAcceleration>(
    velmath_left.getAccel(), 
    velmath_right.getAccel(), 
    (velmath_left.getAccel() + velmath_right.getAccel()) * .5
  );
}

// tare angles
void Lift::tare_angle(QAngle val) {
  m_reference_pose_left =  val - m_absolute_pose_left;
  m_reference_pose_right = val - m_absolute_pose_right;
  update_angles();
}

// update angles
void Lift::update_angles() {
  m_absolute_pose_left = m_motor_left ->getPosition()  * 1_deg / 5.0;
  m_absolute_pose_right = m_motor_right->getPosition() * 1_deg / 5.0;
  velmath_left.step(m_absolute_pose_left.convert(degree));
  velmath_right.step(m_absolute_pose_right.convert(degree));
  m_pose_left = m_absolute_pose_left + m_reference_pose_left;
  m_pose_right = m_absolute_pose_right + m_reference_pose_right;
}