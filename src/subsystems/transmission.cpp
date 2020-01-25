#include "subsystems/transmission.hpp"
#include "subsystems/chassis.hpp"
#include "subsystems/tilter.hpp"
#include "subsystems/lift.hpp"
#include <iostream>

Transmission::Transmission(
  int8_t mtr_direct_left,
  int8_t mtr_direct_right,
  int8_t mtr_shared_left,
  int8_t mtr_shared_right
):

  // init motors
  m_motor_left_direct  (std::make_unique<Motor>(mtr_direct_left,  true,  Motor::gearset::red, Motor::encoderUnits::degrees)),
  m_motor_right_direct (std::make_unique<Motor>(mtr_direct_right, false, Motor::gearset::red, Motor::encoderUnits::degrees)),
  m_motor_left_shared  (std::make_unique<Motor>(mtr_shared_left,  true,  Motor::gearset::red, Motor::encoderUnits::degrees)),
  m_motor_right_shared (std::make_unique<Motor>(mtr_shared_right, false, Motor::gearset::red, Motor::encoderUnits::degrees)),

  // init IMEs
  m_ime_left_direct  (std::make_shared<IntegratedEncoder>(mtr_direct_left,  true)),
  m_ime_right_direct (std::make_shared<IntegratedEncoder>(mtr_direct_right, false)),
  m_ime_left_shared  (std::make_shared<IntegratedEncoder>(mtr_shared_left,  true)),
  m_ime_right_shared (std::make_shared<IntegratedEncoder>(mtr_shared_right, false)),

  // init state
  m_state(State::PASSIVE),

  // init voltages
  m_desired_chassis_voltage_left(0),
  m_desired_chassis_voltage_right(0),
  m_desired_tilter_voltage(0),

  // transmission holding controller
  m_hold_controller(IterativeControllerFactory::posPID(1, 0, 0))
{
  m_hold_controller.setTarget(0);
}


// set chassis reference
void Transmission::set_chassis(std::shared_ptr<Chassis> chassis) {
  m_chassis = chassis;
}

// set tilter reference
void Transmission::set_tilter(std::shared_ptr<Tilter> tilter) {
  m_tilter = tilter;
}

// set lift reference
void Transmission::set_lift(std::shared_ptr<Lift> lift) {
  m_lift = lift;
}

// set the state
void Transmission::set_state(State state) {
  m_state = state;
}

// update the controllers
void Transmission::update() {

  std::cout << (m_tilter->get_angle()).convert(degree) << std::endl;

  // update state
  if (m_state == State::RETRACTING && (m_tilter->get_angle() <= TILTER_RETRACT_THRESHOLD || std::get<2>(m_lift->get_angle()) < Lift::MAX_LOCK)) {
    m_state = State::HOLDING;
    m_hold_controller.setTarget(0);
  }
  if (m_state == State::EXTENDING  && (m_tilter->get_angle() >= TILTER_EXTEND_THRESHOLD || std::get<2>(m_lift->get_angle()) < Lift::MAX_LOCK)) {
    m_state = State::HOLDING;
    m_hold_controller.setTarget(TILTER_RETRACT_THRESHOLD.convert(degree));
  }
  if (m_state == State::HOLDING && m_tilter->get_angle() <= TILTER_RETRACT_THRESHOLD && std::get<2>(m_lift->get_angle()) < Lift::MAX_LOCK) m_state = State::PASSIVE;
  if (m_state == State::PASSIVE && std::get<2>(m_lift->get_angle()) >= Lift::MAX_LOCK) m_state = State::HOLDING;

  // update motors
  switch (m_state) {

    case (State::PASSIVE): 
      m_motor_left_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_right_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_left_direct->moveVoltage(m_desired_chassis_voltage_left);
      m_motor_right_direct->moveVoltage(m_desired_chassis_voltage_right);
      m_motor_left_shared->moveVoltage(m_desired_chassis_voltage_left);
      m_motor_right_shared->moveVoltage(m_desired_chassis_voltage_right);
      break;

    case (State::EXTENDING):
      m_motor_left_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_right_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_left_direct->moveVoltage(m_desired_chassis_voltage_left);
      m_motor_right_direct->moveVoltage(m_desired_chassis_voltage_right);
      m_motor_left_shared->moveVoltage(-12000);
      m_motor_right_shared->moveVoltage(-12000);
      break;

    case (State::RETRACTING):
      m_motor_left_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_right_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_left_direct->moveVoltage(m_desired_chassis_voltage_left);
      m_motor_right_direct->moveVoltage(m_desired_chassis_voltage_right);
      m_motor_left_shared->moveVoltage(12000);
      m_motor_right_shared->moveVoltage(12000);
      break;

    case (State::LOCKED_PASSTHROUGH):
      m_motor_left_direct->setBrakeMode(Motor::brakeMode::hold);
      m_motor_right_direct->setBrakeMode(Motor::brakeMode::hold);
      m_motor_left_direct->moveVelocity(0);
      m_motor_right_direct->moveVelocity(0);
      m_motor_left_shared->moveVoltage(m_desired_tilter_voltage);
      m_motor_right_shared->moveVoltage(m_desired_tilter_voltage);
      break;

    case (State::HOLDING): {
      double correct = m_hold_controller.step(m_tilter->get_angle().convert(degree));
      int shared_voltage_left =  m_desired_chassis_voltage_left  + correct * TILTER_HOLD_STRENGTH;
      int shared_voltage_right = m_desired_chassis_voltage_right - correct * TILTER_HOLD_STRENGTH;

      double scale = 1;
      if (std::abs(shared_voltage_left) > 12000 || std::abs(shared_voltage_right) > 12000) 
        scale = std::min(12000.0 / std::abs(shared_voltage_left), 12000.0 / std::abs(shared_voltage_right));
      
      m_motor_left_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_right_direct->setBrakeMode(Motor::brakeMode::coast);
      m_motor_left_direct->moveVoltage(m_desired_chassis_voltage_left * scale);
      m_motor_right_direct->moveVoltage(m_desired_chassis_voltage_right * scale);
      m_motor_left_shared->moveVoltage(shared_voltage_left * scale);
      m_motor_right_shared->moveVoltage(shared_voltage_right * scale);
    }
  }
}