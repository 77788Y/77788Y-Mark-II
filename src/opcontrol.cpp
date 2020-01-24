#include "main.h"
#include "subsystems/subsystems.hpp"
#include "controls.hpp"

using namespace subsystems;

void opcontrol() {
  while(true) {

    // update poses
    notify_updater(NOTIFY_UPDATE_POSE);

    // control transmission
    if (transmission->m_control_mutex.take(0)) {

      // drive chassis
      int volt_left  = controls::controller_master.getAnalog(ControllerAnalog::leftY)  * 12000;
      int volt_right = controls::controller_master.getAnalog(ControllerAnalog::rightY) * 12000;
      chassis->move_voltage(volt_left, volt_right);

      // control tilter
      if (controls::btn_tilter_extend.isPressed())  tilter->extend_passive();
      else if (controls::btn_tilter_retract.isPressed()) tilter->retract_passive();

      transmission->m_control_mutex.give();
    }

    // control intake
    if (controls::btn_intake_in.isPressed())  intake->move_voltage(12000);
    else if (controls::btn_intake_out.isPressed()) intake->move_voltage(-12000);
    else intake->lock();


    pros::delay(10);
  }
}