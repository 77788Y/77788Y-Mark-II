#pragma once

#include "main.h"
#include "subsystems/tilter.hpp"
#include "subsystems/lift.hpp"

/**
 * Tilter controller.
 * Use when depositing.
 */
class TilterController {

  public:

  /**
   * Constructor.
   * 
   * \param tilter
   *        A reference to the tilter being controlled
   * \param transmission
   *        A reference to the transmission the tilter belongs to
   * \param kp
   *        The kP constant of the PID controller
   * \param ki
   *        The kI constant of the PID controller
   * \param kd
   *        The kD constant of the PID controller
   */
  TilterController(std::shared_ptr<Tilter> tilter, std::shared_ptr<Transmission> transmission, std::shared_ptr<Lift> lift, double kp, double ki, double kd);


  /**
   * Enable the controller.
   */
  void enable();

  /**
   * Disable the controller.
   */
  void disable();

  private:

  /**
   * Should the controller be enabled?
   */
  bool m_enabled;

  /**
   * Reference to the subsystems being controlled.
   */
  std::shared_ptr<Tilter> m_tilter;
  std::shared_ptr<Transmission> m_transmission;
  std::shared_ptr<Lift> m_lift;

  /**
   * The PID controller.
   */
  std::unique_ptr<IterativePosPIDController> m_controller;

  /**
   * The controller task.
   */
  std::unique_ptr<pros::Task> m_task;

};