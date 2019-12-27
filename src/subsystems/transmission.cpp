#include "main.h"
#include "subsystems/transmission.hpp"
#include "./transmission_state_machine.cpp"

namespace transmission {

  // desired voltages of chassis
  int16_t m_left_voltage_desired  = 0;
  int16_t m_right_voltage_desired = 0;

  // motors
  pros::Motor m_left_direct_motor         (1, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor m_right_direct_motor        (2, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor m_left_transmission_motor   (3, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);
  pros::Motor m_right_transmission_motor  (4, pros::E_MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_DEGREES);


  BehaviorType get_behavior() {
    return state_machine.get_behavior();
  }

  StateType get_state() {
    return state_machine.get_state();
  }

  void move_voltage(int left, int right) {
    m_left_voltage_desired  = left;
    m_right_voltage_desired = right;
  }

  void move_voltage(int val) {
    move_voltage(val, val);
  }

   void deposit_stack() {
     state_machine.change_state(StateType::DEPOSITED);
   }

   void retract_tray() {
     state_machine.change_state(StateType::LOCKED);
   }
}