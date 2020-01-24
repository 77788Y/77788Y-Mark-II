#pragma once

#include "subsystems/chassis.hpp"
#include "subsystems/transmission.hpp"
#include "subsystems/tilter.hpp"

namespace subsystems {

  /**
   * Task notification types.
   */
  enum UpdaterNotification {

    NOTIFY_UPDATE_POSE, ///< Update the poses of the subsystems
    NOTIFY_UPDATE_CONT  ///< Update the controllers of the subsystems.
  };

  /**
   * Send a notification to the updater task.
   * 
   * \param notification
   *        The type of notification to send
   */
  void notify_updater(UpdaterNotification notification);

  /**
   * Update task.
   * When in autonomous, this will run automatically.
   * When in opcontrol, send this task NOTIFY_UPDATE_POSE to update poses.
   * Whe in opcontrol, send NOTIFY_UPDATE_CONT to update controllers.
   * This will only update subsystems if their mutex is available.
   * Some controllers may prefer to themselves handle updating.
   */
  extern std::shared_ptr<pros::Task> updater_task;

  /**
   * Subsystem objects.
   * These should be the only objects created from the subsystem classes.
   * These should probably be singletons but I didn't bother learning how that works.
   */
  extern std::shared_ptr<Transmission> transmission; ///< Transmission object
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