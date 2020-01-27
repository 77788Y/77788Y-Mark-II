#include "main.h"
#include "subsystems/subsystems.hpp"
#include "controllers/controllers.hpp"
#include "controls.hpp"

using namespace subsystems;

void opcontrol() {
  while(true) {

    // update poses
    update_poses();

    // control transmission
    // if (controls::btn_tilter_slow.changedToPressed()) tilter->move_voltage(-6000);
    // else if (controls::btn_tilter_slow.changedToReleased()) tilter->hold();
    // else if (controls::btn_tilter_macro_deposit.changedToPressed()) subsystem_controllers::tilter_controller->enable();
    // else if (controls::btn_tilter_macro_deposit.changedToReleased()) subsystem_controllers::tilter_controller->disable();
    // if (controls::btn_tilter_pull_out.changedToPressed()) subsystem_controllers::pull_out_controller->enable();
    // else if (controls::btn_tilter_pull_out.changedToReleased()) subsystem_controllers::pull_out_controller->disable();

    if (transmission->m_control_mutex.take(0)) {

      // drive chassis
      int volt_left  = controls::controller_master.getAnalog(ControllerAnalog::leftY)  * 12000;
      int volt_right = controls::controller_master.getAnalog(ControllerAnalog::rightY) * 12000;
      chassis->move_voltage(volt_left, volt_right);

      // control tilter
      // if (controls::btn_tilter_extend.isPressed())  tilter->extend_passive();
      // else if (controls::btn_tilter_retract.isPressed()) tilter->retract_passive();

      transmission->m_control_mutex.give();
    }

    // control intake
    if (controls::btn_intake_in.isPressed() && controls::btn_intake_out.isPressed()) intake->move_voltage(-4000);
    else if (controls::btn_intake_in.isPressed()) {
      intake->move_voltage(12000);
      // subsystem_controllers::lift_controller->lower();
    }
    else if (controls::btn_intake_out.isPressed()) {
      intake->move_voltage(-12000);
      // subsystem_controllers::lift_controller->lower();
    }
    else {
      intake->lock();
      // subsystem_controllers::lift_controller->raise();
    }

    // control lift
    if (controls::btn_lift_up.isPressed()) lift->move_voltage(12000);
    else if (controls::btn_lift_down.isPressed()) lift->move_voltage(-8000);
    else lift->lock();

    // update controllers
    update_controllers();

    pros::delay(10);
  }
}