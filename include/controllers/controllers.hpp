#pragma once

#include "controllers/tilter_controller.hpp"
#include "controllers/pull_out_controler.hpp"
#include "controllers/lift_controller.hpp"

namespace subsystem_controllers {

  /**
   * Subsystem objects.
   * These should be the only objects created from the subsystem classes.
   * These should probably be singletons but I didn't bother learning how that works.
   */
  extern std::shared_ptr<TilterController> tilter_controller;
  extern std::shared_ptr<PullOutController> pull_out_controller;
  // extern std::shared_ptr<LiftController> lift_controller;

  /**
   * Initialize all subsystems.
   * Should be run before any references to them.
   */
  void init();
}