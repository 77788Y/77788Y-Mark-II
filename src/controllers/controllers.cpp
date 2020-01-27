#include "subsystems/subsystems.hpp"
#include "controllers/controllers.hpp"

namespace subsystem_controllers {

  /**
   * Subsystem objects.
   * These should be the only objects created from the subsystem classes.
   * These should probably be singletons but I didn't bother learning how that works.
   */
  std::shared_ptr<TilterController> tilter_controller;
  std::shared_ptr<PullOutController> pull_out_controller;
  // std::shared_ptr<LiftController> lift_controller;

  /**
   * Initialize all subsystems.
   * Should be run before any references to them.
   */
  void init() {
    tilter_controller = std::make_shared<TilterController>(subsystems::tilter, subsystems::transmission, subsystems::lift, .015, 0, .005);
    pull_out_controller = std::make_shared<PullOutController>(subsystems::tilter, subsystems::chassis, subsystems::transmission, subsystems::intake);
    // lift_controller = std::make_shared<LiftController>(subsystems::lift, subsystems::intake, .01, 0, 0);
  }
}