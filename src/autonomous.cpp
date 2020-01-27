#include "main.h"
#include "subsystems/subsystems.hpp"
#include "controllers/controllers.hpp"

using namespace subsystems;
// using namespace subsystem_controllers;

void autonomous() {

  
  // push cube in
  chassis->move_voltage(8000);
  pros::delay(750);
  chassis->move_voltage(0);
  pros::delay(500);
  intake->move_voltage(-12000);
  pros::delay(500);
  chassis->move_voltage(-8000);
  pros::delay(750);
  chassis->move_voltage(0);

  // // flip out
  // tilter_controller->enable();
  // pros::delay(1000);
  // chassis->move_voltage(-8000);
  // pros::delay(500);
  // chassis->move_voltage(0);
  // pros::delay(500);
  // tilter->retract_passive();
  // pros::delay(5000);
  

}