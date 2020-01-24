#include "subsystems/chassis.hpp"

// constructor
Chassis::Chassis(std::shared_ptr<Transmission> transmission, std::unique_ptr<Odom> odom):
  m_transmission(transmission), m_odom(std::move(odom)) {
    // m_transmission->m_chassis = std::make_shared<Chassis>(this);
  }

// move voltage
void Chassis::move_voltage(int l, int r) {
  m_transmission->m_desired_chassis_voltage_left = l;
  m_transmission->m_desired_chassis_voltage_right = r;
}
void Chassis::move_voltage(int val) {
  m_transmission->m_desired_chassis_voltage_left = val;
  m_transmission->m_desired_chassis_voltage_right = val;
}

// get pose
Odom::ChassisPose* Chassis::get_pose() {
  return m_odom->get_pose();
}
Odom::ChassisDeriv* Chassis::get_speed() {
  return m_odom->get_speed();
}

// tare the pose
void Chassis::tare_pose(Odom::ChassisPose* new_pose) {
  m_odom->tare(new_pose);
}

// update the pose
void Chassis::update_pose() {
  m_odom->update();
}