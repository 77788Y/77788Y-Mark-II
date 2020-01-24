#pragma once

#include "main.h"

namespace controls {

  // controller
  inline Controller controller_master;

  // chassis
  inline ControllerButton btn_chassis_brake(ControllerDigital::left);

  // tilter
  inline ControllerButton btn_tilter_macro_deposit(ControllerDigital::A);
  inline ControllerButton btn_tilter_extend(ControllerDigital::X);
  inline ControllerButton btn_tilter_retract(ControllerDigital::Y);

  // intake
  inline ControllerButton btn_intake_in (ControllerDigital::R1);
  inline ControllerButton btn_intake_out(ControllerDigital::R2);

  // lift
  inline ControllerButton btn_lift_up  (ControllerDigital::L1);
  inline ControllerButton btn_lift_down(ControllerDigital::L2);
}