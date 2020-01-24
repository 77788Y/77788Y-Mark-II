#pragma once

#include "subsystems/chassis.hpp"
#include "subsystems/transmission.hpp"
#include "subsystems/tilter.hpp"

namespace subsystems {

  /**
   * Subsystem objects.
   * These should be the only objects created from the subsystem classes.
   * These should probably be singletons but I didn't bother learning how that works.
   */
  extern std::shared_ptr<Chassis> chassis; ///< Chassis object
  extern std::shared_ptr<Tilter>  tilter;  ///< Tilter object

  /**
   * Initialize all subsystems.
   * Should be run before any references to them.
   */
  void init();

  /**
   * Update the pose of all subsystem objects.
   * Should be run before operating on subsystems.
   */
  void update_poses();

  /**
   * Update the controllers of all subsystem objects.
   * Should be run after operating on subsystems (not before).
   */
  void update_controllers();
}