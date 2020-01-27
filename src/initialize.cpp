#include "main.h"
#include "subsystems/subsystems.hpp"
#include "controllers/controllers.hpp"

void initialize() {

  // init subsystems
  subsystems::init();

  // init controllers
  subsystem_controllers::init();
}

void competition_initialize() {};

void disabled();